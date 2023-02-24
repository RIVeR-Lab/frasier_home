import time
import onscreen

def verify_command(self):
	perceived_cmd = self.sentence
	repeat_cmd = perceived_cmd.replace(' me ',' you ').replace(' me.',' you.')
	txt = 'You asked me to '+repeat_cmd
	self.speech.say(txt)
	txt = 'Is this 100 percent correct?'
	self.speech.say(txt)
	txt = 'If not, I will ask my mommy or daddy for help.'
	self.speech.say(txt)
	response = is_correct(self)
	if response is None or not response:
		txt = 'Sorry about that. Waiting for custom operator.'
		self.speech.say(txt)
		get_response(self)
		return verify_command(self)
	return perceived_cmd


def report_num_people_from_pose_msgs(self, pose_msgs):
    num_ppl = len(pose_msgs)
    if num_ppl > 1:
        self.speech.say('I found ' + str(num_ppl) + 'people. I need everyone to stay still and face me.')
    # elif num_ppl == 1:
    #     self.speech.say("I found one person. Please stay still and face me.")


# def verify_cmd(self,cmd):
# 	repeat_cmd = cmd.replace(' me',' you')
#     txt = "You asked me to "+repeat_cmd +\
#           ". Is that correct?" +\
#     response = get_response(self)
#     if response == 'correct'

def is_correct(self):
    self.speech.say("Please say HSR that's correct, or HSR that's incorrect")
    response = get_response(self)
    if response == 'correct':
        return True
    elif response == 'incorrect':
        return False
    else:
        return None


def get_response(self, count=2, only_last_word=True):
    # Wait 10 seconds for response
    timer = time.time()
    self.result_rcvd = False
    while not self.result_rcvd and time.time() < timer + 10:
        pass
    if self.result_rcvd:
        if only_last_word:
            response = self.sentence.split()[-1]
        else:
            response = self.sentence
    else:
        response = ''
    self.result_rcvd = False
    self.listening_for_response = False

    count -= 1

    if response != '' or count <= 0:
        return response
    else:
        self.speech.say('Please respond.')
        return get_response(self, count)

def speak_with_subs(self,txt,blocking=True):
    onscreen.subtitles(txt)
    self.speech.say(txt,blocking=blocking)
