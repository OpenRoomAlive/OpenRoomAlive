#!/usr/bin/env python

from Tkinter import *

from runcommon import runProgram

def main():

  # Main window.
  root = Tk()
  root.title("ProCam node")

  # Left frame - for choosing options.
  frameLeft = Frame(root)
  frameLeft.grid(row=0, column=0, sticky=N+S)


  #### Left  frame #############################################################

  # Choosing display resolution.
  resLabel = Label(frameLeft, text="Set the display resolution (widthxheight):")
  resLabel.grid(row=0, columnspan=3, padx=(5, 5), sticky=W)
  resEnt = Entry(frameLeft)
  resEnt.config(width=15)
  resEnt.insert(0, "1024x768")
  resEnt.grid(row=0, column=3, padx=(5, 5))

  # Choosing IP of the master node.
  ipLabel = Label(frameLeft, text = "Set IP of the master node:")
  ipLabel.grid(row=1, columnspan=3, padx=(5, 5), sticky=W)
  ipEnt = Entry(frameLeft)
  ipEnt.insert(0, "localhost")
  ipEnt.config(width=15)
  ipEnt.grid(row=1, column=3, padx=(5, 5))


  #### Run #####################################################################
  rowForButtons = 2
  buttonTexts = [
      "Run the ProCam",
      "Shut it down",
      "The ProCam was shut down."]
  program = "../build/procam/procam"
  widgets = [(resEnt, "effective-size"), (ipEnt, "ip")]


  runProgram(root, frameLeft, rowForButtons, buttonTexts, program, widgets)

if __name__ == '__main__':
  main()
