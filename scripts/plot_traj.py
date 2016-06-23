#!/usr/bin/python

import sys
import os
from PIL import Image
import glob

def plot_traj(filename):
  print "plot traj: "+filename
  f=open(filename,'r')
  title=filename+" "+f.readline()[2:].rstrip()
  sizex,sizey=map(int,str.split(f.readline())[3:5])
  if (sizex != sizey):
    sys.exit("ERROR: sizex!=sizey: sizex="+str(sizex)+" sizey="+str(sizey))
  mapname=f.readline()[2:].rstrip()
  print title+" size="+str(sizex)+" mapname="+mapname
  mappng="/tmp/map.png"
  os.system("convert "+mapname+" "+mappng)
  im=Image.open(mapname)
  #print "Image size: "+str(im.size) # (width,height) tuple
  if (im.size[0] != im.size[1]):
    sys.exit("ERROR: image size incorrect: sizex!=sizey: sizex="+str(im.size[0])+" sizey="+str(im.size[1])+" for image: "+mappng+" png version of "+mapname)

  ratio=sizex/float(im.size[0])
  #for l in f.readlines():
  fplot=open(filename+".plot",'w')
  fplot.write("set title '"+title+"'\n")
  fplot.write("plot [0:"+str(im.size[0])+"][0:"+str(im.size[1])+"]'"+mappng+"' binary filetype=png with rgbimage\n")
  #fplot.write("replot '"+filename+"' u ($1/"+str(ratio)+"):("+str(im.size[1])+"-$2/"+str(ratio)+") w p notitle\n")
  fplot.write("replot '"+filename+"' u ($1/"+str(ratio)+"):("+str(im.size[1])+"-$2/"+str(ratio)+") w l notitle\n")
  fplot.write("pause -1\n")
  fplot.close()
  os.system("gnuplot "+filename+".plot")

if __name__ == '__main__':
  #print "Calling plot_traj with args: "+str(sys.argv)
  if (len(sys.argv)<2):
    print("Usage: "+sys.argv[0]+" <trajectory file name>")
  else:
    for f in sys.argv[1:]:
        f1=glob.glob(f)
        for ff in f1:
            plot_traj(f)
