#!/usr/bin/env python2

import cv2 as cv
import math
import numpy as np
import random
import sys

from itertools import izip, islice
from scipy.cluster.vq import kmeans, whiten

WIDTH = 640
HEIGHT = 480
WINDOW = 50
TRESHOLD = 30

def dist(a, b):
  """Computes the distance between two points."""

  dx = a[0] - b[0]
  dy = a[1] - b[1]
  return math.sqrt(dx * dx + dy * dy)


def main(path):
  """Entry point of the application."""

  cap = cv.VideoCapture(path)

  points = []
  frames = []
  candidates = []
  track = None
  prev = np.empty([HEIGHT, WIDTH], np.uint8)
  while cv.waitKey(1) & 0xFF != ord('q'):
    ret, frame = cap.read()
    if not ret:
      break

    # Accumulate the last 3 images.
    frame = cv.cvtColor(cv.resize(frame, (WIDTH, HEIGHT)), cv.COLOR_BGR2GRAY)

    if track is not None:
      # If we had a previous position, try to restrict the search area
      # to a 50x50 window around the old position, improving speed
      # and resiliency to noise.
      offY = max(track[0] - WINDOW, 0)
      limY = min(HEIGHT - 1, track[0] + WINDOW)
      offX = max(track[1] - WINDOW, 0)
      limX = min(WIDTH - 1,  track[1] + WINDOW)
      wnd_frame = frame[offY:limY, offX:limX]
      wnd_prev = prev[offY:limY, offX:limX]

      # Compute the difference between two consecutive frames and find
      # the point of maximal intensity in the search window. Noisy points
      # are removed using tresholding since we prefer to loose track and
      # find the pointer again to following noise.
      diff = cv.absdiff(wnd_frame, wnd_prev)
      _, diff = cv.threshold(diff, TRESHOLD, 255, cv.THRESH_TOZERO)

      track = np.unravel_index(np.argmax(diff), diff.shape)
      if track[0] > 0 and track[1] > 0:
        track = (track[0] + offY, track[1] + offX)
        points.append(track)
      else:
        track = None

    if not track:
      # Compute the difference & treshold for noise.
      diff = cv.absdiff(frame, prev)
      _, diff = cv.threshold(diff, TRESHOLD, 255, cv.THRESH_TOZERO)

      # Find the points of great intensity.
      maxv = np.unravel_index(np.argmax(diff), diff.shape)
      to_track = list(zip(*np.where(diff >= diff[maxv] / 2)))
      if diff[maxv] > TRESHOLD and len(to_track) < 50:
        new_candidates = []
        old = []

        # Keep a set of candidate points. If in the new frame does not
        # have any points close to the old candidates, discard them.
        # If there are neighbours, increase the 'age' of the old points.
        for point, age in candidates:
          count = 0
          for new_point in to_track:
            if dist(point, new_point) < 5:
              count += 1
          if count > 0:
            new_candidates.append((point, age + count))
          if age + count > 2:
            old.append((point, age))
        candidates = new_candidates + [(x, 0) for x in to_track]

        # Select the candidate which belongs to the most populous cluster.
        sorted(old, key=lambda pt: pt[0])
        if len(old) > 0:
          track = old[0][0]


    # Draw the detected path.
    draw = np.copy(frame)
    for p0, p1 in izip(points, islice(points, 1, None)):
      dx = p0[1] - p1[1]
      dy = p0[0] - p1[0]
      line_length = math.sqrt(dx * dx + dy * dy)
      if line_length < 50:
        cv.line(draw, (p0[1], p0[0]), (p1[1], p1[0]), (0, 255, 0), 1)

    prev = frame
    cv.imshow('draw', draw)

  cap.release()
  cv.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv[1] if len(sys.argv) >= 2 else 0)
