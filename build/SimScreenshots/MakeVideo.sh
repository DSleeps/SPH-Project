./ffmpeg -r 30 -f image2 -s 1920x1080 -i Sim%d.bmp -vcodec libx264 -crf 5 -pix_fmt yuv420p output.mp4
rm Sim*
