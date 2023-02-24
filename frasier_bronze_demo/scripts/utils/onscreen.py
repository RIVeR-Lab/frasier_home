# ROS
import rospy
import shutil
import time
import os	
import cv2

dirname = os.path.dirname(os.path.abspath(__file__))+'/site/'

def change_img(imgpath):
	shutil.copy(imgpath,dirname+'current_img.jpg')

def change_main_html(newdata):
	f = open(dirname+'index.html','w')
	f.write(newdata)
	f.close()

def show_command(cmd):
	htmlpath = dirname+'confirm_cmd.html'
	newdata = replace_file_content(htmlpath,'COMMAND',cmd)
	change_main_html(newdata)

def replace_file_content(filepath,find,replace):
	f = open(filepath,'r')
	filedata = f.read()
	f.close()
	newdata = filedata.replace(find,replace)
	return newdata

def subtitles(txt):
	htmlpath = dirname+'subtitles.html'
	newdata = replace_file_content(htmlpath,'SUBTITLES',txt)
	change_main_html(newdata)

def show_image(img):
	cv2.imwrite(dirname+'current_img.jpg',img)
	htmlpath = dirname+'just_img.html'
	newdata = replace_file_content(htmlpath,'','')
	change_main_html(newdata)

def show_desc_txts(txts):
	print txts
	if not txts:
		return
	htmlpath = dirname+'descs.html'
	newdata = replace_file_content(htmlpath,'DESCS','<br><br>'.join(txts))
	change_main_html(newdata)
