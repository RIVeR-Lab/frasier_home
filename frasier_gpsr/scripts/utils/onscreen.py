# ROS
import rospy
import shutil
import time
import os	

dirname = os.path.dirname(os.path.abspath(__file__))

def change_img(imgpath):
	shutil.copy(imgpath,dirname+'/templates/current_img.jpg')

def change_html(htmlpath):
	shutil.copy(htmlpath,dirname+'/templates/index.html')

def show_command(cmd):
	htmlpath = dirname+'/templates/confirm_cmd.html'
	replace_file_content(htmlpath,'COMMAND',cmd)
	change_html(htmlpath)

def replace_file_content(filepath,find,replace):
	f = open(filepath,'r')
	filedata = f.read()
	f.close()
	newdata = filedata.replace(find,replace)
	f = open(filepath,'w')
	f.write(newdata)
	f.close()

def subtitles(self,txt):
	htmlpath = dirname+'/templates/subtitles.html'
	replace_file_content(htmlpath,'SUBTITLES',txt)
	change_html(htmlpath)
	self.speech.say(txt)

def set_default():
	change_html(dirname+'/templates/default.html')

if __name__ == '__main__':
	change_html(dirname+'/templates/default.html')
	time.sleep(2)
	change_html(dirname+'/templates/confirm_cmd.html')
	time.sleep(2)
	change_html(dirname+'/templates/just_img.html')
	time.sleep(2)