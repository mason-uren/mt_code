#!/bin/sh

tmux start-server
tmux new-session -d -n 'PTU_Server'
tmux send-keys -t PTU_Server './Run_PTU.sh' Enter

tmux new-window -d -n 'Imperx_Server'
tmux send-keys -t Imperx_Server './Run_Imperx.sh' Enter

tmux new-window -d -n 'Ximea_Server'
tmux send-keys -t Ximea_Server './Run_Ximea.sh' Enter

tmux -2 attach-session -d

