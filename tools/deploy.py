#!/usr/bin/env python2

import logging
import os
import socket
import subprocess
import sys
import threading

import paramiko



# Git URL.
GIT = 'git@gitlab.doc.ic.ac.uk:nl1813/DerpVision.git'

# List of hosts that will run ProCam units.
HOSTS = [
  ('nandor', 'localhost', '/Users/nandor/build1'),
  ('nandor', 'localhost', '/Users/nandor/build2')
]



class Task(object):
  """Buildable & runnable task."""

  def __init__(self, name):
    """Initializes the task."""

    self.name = name
    self.logger = logging.getLogger(self.name)


class ProCam(Task):
  """Class handling the connection to a ProCam unit."""

  def __init__(self, user, host, path, sha, master_ip):
    """Initializes the connection."""

    super(ProCam, self).__init__(host)

    self.sha = sha
    self.master_ip = master_ip
    self.path = path

    self.logger.info('Connecting...')
    self.ssh = paramiko.SSHClient()
    self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    self.ssh.connect(host, username=user)
    self.logger.info('Connected to host.')

  def build(self):
    """Checks the repository out & builds the procam client."""

    build_dir = os.path.join(self.path, 'build')

    self.logger.info('Build started...')

    self.run_command('rm -rf %s' % self.path)
    self.run_command('git clone %s %s' % (GIT, self.path))
    self.run_command('mkdir -p %s' %  build_dir)
    self.run_command('cmake ..', build_dir)
    self.run_command('make procam -j4', build_dir)

  def run(self):
    """Runs the program client."""

    self.logger.info('Running ProCam client.')

    self.run_command('build/procam/procam --ip %s' % self.master_ip, self.path)

  def run_command(self, command, cwd='.'):
    """Runs a single command."""

    self.logger.info(command)

    stdin, stdout, stderr = self.ssh.exec_command(
        'source ~/.zshrc; cd %s; %s' % (cwd, command)
    )
    if stdout.channel.recv_exit_status():
      for line in stderr.read().splitlines():
        if line:
          self.logger.error(line)
        raise RuntimeError('Cannot execute command.')


class Master(Task):
  """Class handling the master server."""

  def __init__(self, sha, hosts):
    """Initializes the master server."""

    super(Master, self).__init__('master')

    self.sha = sha
    self.hosts = hosts

  def build(self):
    """Runs make, assuming tool is started from the root directory."""

    self.logger.info('Build started...')

    self.run_command('git checkout %s' % self.sha)
    self.run_command('mkdir -p build', '.')
    self.run_command('cmake ..', 'build')
    self.run_command('make -C build master', '.')

    self.logger.info('Build succeeded.')

  def run(self):
    """Runs the master server."""

    self.logger.info('Master started...')

    self.run_command('build/master/master --procam-total %d' % len(self.hosts))

  def run_command(self, command, cwd='.'):
    """Runs a single command."""

    self.logger.info(command)

    p = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True,
        cwd=cwd
    )

    stdout, stderr = p.communicate()
    if p.returncode:
      for line in stderr.splitlines():
        if line:
          self.logger.error(line)
      raise subprocess.CalledProcessError(p.returncode)



def main():
  """Entry point of the script."""

  # Set up logging.
  logging.basicConfig(stream=sys.stdout, level=logging.INFO)
  logging.getLogger().name = 'deploy'
  logging.getLogger('paramiko').setLevel(logging.ERROR)

  # Read in the SHA to build.
  try:
    with open('.deploy') as f:
      sha = f.read()
  except:
    logging.error('Deploy script must be run from project root.')
    sys.exit(-1)
  logging.info('Deploying %s.', sha)

  # Get the IP of master.
  master_ip = None
  for ip in socket.gethostbyname_ex(socket.gethostname())[2]:
    if not ip.startswith("127."):
      master_ip = ip
      break
  logging.info('Master IP: %s' % master_ip)

  # Create all the tasks to run.
  logging.info('Connecting to all hosts...')
  procams = []
  master = Master(sha, HOSTS)
  for user, host, path in HOSTS:
    try:
      procams.append(ProCam(user, host, path, sha, master_ip))
    except:
      logging.error('Cannot connect to %s' % host)
      sys.exit(-1)

  # Build everything.
  logging.info('Building on all hosts...')
  for procam in procams:
    try:
      procam.build()
    except Exception as e:
      logging.error('Cannot build %s: %s' % (procam.name, e))
      sys.exit(-1)
  master.build()

  # Run everything.
  logging.info('Running on all hosts...')
  for procam in procams:
    try:
      t = threading.Thread(target = lambda: procam.run())
      t.start()
    except Exception as e:
      logging.error('Cannot run %s: %s' % (procam.name, e))
      sys.exit(-1)
  master.run()


if __name__ == '__main__':
  main()
