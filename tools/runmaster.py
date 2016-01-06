#!/usr/bin/env python

from Tkinter import *

from runcommon import runProgram

def main():

  # Main window.
  root = Tk()
  root.title("Master node")

  # Left frame - for choosing options.
  frameLeft = Frame(root)
  frameLeft.grid(row=0, column=0, sticky=N+S)


  #### Left  frame #############################################################

  # Choosing procams number.
  procamLabel = Label(frameLeft, text="Set the number of ProCams you use:")
  procamLabel.grid(row=0, columnspan=3, padx=(5, 5), sticky=W)
  procamVar = StringVar()
  procamVar.set('1')
  procamMenu = OptionMenu(frameLeft, procamVar,
      '1', '2', '3', '4', '5', '6', '7')
  procamMenu.config(width=10)
  procamMenu.grid(row=0, column=3, padx=(5, 5))

  # Choosing whether calibration needed.
  calibLabel = Label(
      frameLeft,
      text = "Does the system need to be recalibrated (it  \n"
             "might have changed since last calibration)? :")
  calibLabel.grid(row=1, columnspan=3, padx=(5, 5), sticky=W)
  calibVar = StringVar()
  calibVar.set('true')
  calibMenu = OptionMenu(frameLeft, calibVar, 'false', 'true')
  calibMenu.config(width=10)
  calibMenu.grid(row=1, column=3, padx=(5, 5))

  # Choosing between 3D rendering and laser painting.
  renderLabel = Label(
      frameLeft,
      text = "Would you like to see the 3D scene           \n"
             "reconstruction instead of laser painting? :")
  renderLabel.grid(row=2, columnspan=3, padx=(5, 5), sticky=W)
  renderVar = StringVar()
  renderVar.set('false')
  renderMenu = OptionMenu(frameLeft, renderVar, 'false', 'true')
  renderMenu.config(width=10)
  renderMenu.grid(row=2, column=3, padx=(5, 5))

  #### Run #####################################################################
  rowForButtons = 3
  buttonTexts = [
      "Run the system",
      "Shut it down",
      "The system was shut down."]
  program = "../build/master/master"
  widgets = [
      (procamVar, "procam-total"),
      (calibVar, "calibrate"),
      (renderVar, "render")]


  runProgram(root, frameLeft, rowForButtons, buttonTexts, program, widgets)

if __name__ == '__main__':
  main()
