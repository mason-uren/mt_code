#!/bin/bash -f
# Script to digest a sequence of lines like this from <stdin>
# Number of marker detected: 23
# Number of marker detected: 20
# ...
# Sum up the numbers and print the total at the end
# E.g., one can use this script on the tracker stand-alone code output
# in debugger mode:
# TrackingFiducial_Example -d -v <input_video_file>  | bash <this_script>
# 3823

grep 'Number of marker detected' $1 | cut -f5 -d' ' | \
  (TOTAL=0; while read C;  do ((TOTAL += C)) ;  done;  echo $TOTAL )
