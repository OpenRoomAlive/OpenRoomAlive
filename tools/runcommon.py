from Tkinter import *
import subprocess
import threading
import time

class StdoutRedirect(object):
    def __init__(self, newWidgetOutput):
        self.newWidgetOutput = newWidgetOutput

    def write(self, text):
        # Insert new text.
        self.newWidgetOutput.config(state='normal')
        self.newWidgetOutput.insert(END, text)
        self.newWidgetOutput.config(state='disabled')
        # Scroll to end.
        self.newWidgetOutput.see(END)


def runProgram(root, frameLeft, rowForButtons, buttonTexts, program, widgets):

  ##### Background process handling. ###########################################

  background = {'process': None, 'stdoutThread': None, 'stderrThread': None}

  def callBackground():
    # Run the background program.
    cmd = program
    for widget in widgets:
      cmd += " --" + widget[1] + "=" + widget[0].get()
    print "Running in the background: " + cmd + "\n\n"
    args = cmd.split()
    background['process'] = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stdin=subprocess.PIPE,
        stderr=subprocess.PIPE)

    # Enable quit button.
    quitButton.config(state='normal')
    # Start thread for fetching stderr of the background program.
    background['stderrThread'] = threading.Thread(target=fetchStderrBackground)
    background['stderrThread'].start()

    # Fetch stdout of the background program.
    for line in iter(background['process'].stdout.readline, ''):
      print line

  def fetchStderrBackground():
    # Fetch stderr of the background program.
    for line in iter(background['process'].stderr.readline, ''):
      print line

  #### Buttons' actions. #######################################################

  def startBackground():
    runButton.config(state='disabled')
    background['stdoutThread'] = threading.Thread(target=callBackground)
    background['stdoutThread'].start()


  def quitBackground():
    shutDown()

    # Message user and reset buttons.
    print("\n" + buttonTexts[2] + "\n-------------------------")
    quitButton.config(state='disabled')
    runButton.config(state='normal')

  def closeApp(event):
    shutDown()
    event.widget.destroy()

  def shutDown():
    # If process is alive, shut it down.
    if background is not None and background['process'] is not None and \
       background['process'].poll() is None:
      # Ask program to do shutdown.
      try:
        background['process'].stdin.write("q")
      except IOError:
        # Program could have been shut down after our check.
        pass

      # Wait for shut down.
      time.sleep(0.5)

      # Kill the process if it is not yet done.
      if background is not None and background['process'] is not None and \
         background['process'].poll() is None:
        background['process'].kill()


  #### Right frame #############################################################

  frameRight = Frame(root)
  frameRight.grid(row=0, column=1, sticky=N+S+E+W)

  # Log label.
  msgLabel = Label(frameRight, text="Message log:")
  msgLabel.grid(row=0, columnspan=2)

  # Text area with scrollbar.
  outputArea = Text(frameRight, bg="#E7FDE5", state='disabled')
  outputArea.bind_all("<1>", lambda event: event.widget.focus_set())
  outputArea.grid(row=1, sticky=N+S+E+W)
  scrl = Scrollbar(frameRight, command=outputArea.yview)
  outputArea.config(yscrollcommand=scrl.set)
  scrl.grid(row=1, column=1, sticky=N+S+E+W)

  # When stretching, give extra space to text area.
  frameRight.columnconfigure(0, weight=1)
  frameRight.rowconfigure(1, weight=1)

  # Stdout/stderr piped to text area.
  sys.stdout = StdoutRedirect(outputArea)


  #### Left  frame #############################################################

  # Start and stop buttons.
  runButton = Button(frameLeft, text=buttonTexts[0], command=startBackground)
  runButton.config(width=10)
  runButton.grid(row=rowForButtons, column=0, columnspan=2, pady=(30, 0),
      padx=(0, 5), sticky=E)
  quitButton = Button(frameLeft, text=buttonTexts[1], command=quitBackground)
  quitButton.config(width=10, state='disabled')
  quitButton.grid(row=rowForButtons, column=2, columnspan=2, pady=(30, 0),
      padx=(5, 0), sticky=W)


  ### Run app. #################################################################

  # Make sure the app is on top of all application windows.
  root.attributes('-topmost', 1)
  # Once clicked (which changes active window), stop forcing topmost (not needed
  # anymore).
  def stopForceTopmost(event):
    root.attributes('-topmost', 0)
    event.widget.focus_set()
  frameLeft.bind_all("<1>", lambda event: stopForceTopmost(event))

  # Make sure background process is killed.
  root.bind("<Destroy>", lambda event: closeApp(event))

  # When stretching, give extra space to right frame.
  root.columnconfigure(1, weight=1)
  # Move away from corner.
  root.geometry("+50+50")

  # Listen to events.
  root.mainloop()
