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
  ('nl1813', 'voxel22.doc.ic.ac.uk', '~/build1'),
  ('nl1813', 'voxel21.doc.ic.ac.uk', '~/build2')
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

    self.run_command('git clone %s' % GIT, self.path)
    self.run_command('mkdir -p %s', build_dir)
    self.run_command('cmake ..', build_dir)
    self.run_command('make slave', build_dir)

  def run(self):
    """Runs the program client."""

    self.logger.info('Running ProCam client.')

    self.run_command('slave/slave --ip %s' % self.master_ip, build_dir)

  def run_command(self, command, cwd='.'):
    """Runs a single command."""

    self.logger.info(command)

    stdin, stdout, stderr = client.exec_command(
        'cd %s; %s', cwd, command
    )
    if stdout.channel.recv_exit_status():
      for line in stderr.read().splitlines():
        if line:
          self.logger.error(line)
      raise subprocess.CalledProcessError(p.returncode)


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

    self.run_command('build/master/master --procamTotal %d' % len(self.hosts))

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
  tasks = []
  tasks.append(Master(sha, HOSTS))
  for user, host, path in HOSTS:
    try:
      tasks.append(ProCam(user, host, path, sha, master_ip))
    except:
      logging.error('Cannot connect to %s' % host)
      sys.exit(-1)

  # Build everything.
  logging.info('Building on all hosts...')
  for task in tasks:
    try:
      task.build()
    except:
      logging.error('Cannot build %s' % task.name)
      sys.exit(-1)

  # Run everything.
  logging.info('Running on all hosts...')
  for task in tasks:
    try:
      task.run()
    except:
      logging.error('Cannot run %s' % task.name)
      sys.exit(-1)


if __name__ == '__main__':
  main()
