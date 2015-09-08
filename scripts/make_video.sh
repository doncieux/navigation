#!/bin/bash

# makes a video from available images (img_XXX.bmp)

if [ $# -lt 1 ]
then
    OUTFILE="video.mpg"
else
    OUTFILE=$1
fi

echo "converting images to jpg"
for a in img_*.bmp
do
    convert $a $(basename $a .bmp).jpg
done

echo "building video ..."
avconv -f image2 -i img_%6d.jpg -r 24 $OUTFILE
#ffmpeg -f image2 -i img_%0.jpg -r 24 -vcodec mpeg4 -b 15000k $OUTFILE
#convert img_*.jpg $OUTFILE
echo "[done]"