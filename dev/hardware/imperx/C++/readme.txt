Imperx Gui:

Issues:
The Imperx API that comes with the camera isnt very well documented. When trying to run the camera without a GUI it will hang when reading a internal buffer. 

Bandaid Solution:
In light of this I had to hack provided GUI. The gui is written in QT. To update the main display there is a thread which reads the camera buffer. I created a seperate thread which reads this buffer and sends the data out using ZMQ (a glorified socket library).

To Run:
