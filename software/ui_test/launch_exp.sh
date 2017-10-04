  310  ls
  311  vlc ./test_vid.mpg 
  312  vlc http://c09.lan/video.mjpg --sout=file/ts:test_vid_09.mpg &
  313  vlc http://c35.lan/video.mjpg --sout=file/ts:test_vid_35.mpg &
  314  ls
  315  vlc test_vid_35.mpg 
  316  vlc test_vid_09.mpg 
  317  ls
  318  history
  319  history | tail > launch_exp.sh
