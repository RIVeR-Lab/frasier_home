# ROS
import rospkg

pkg_dir = rospkg.RosPack().get_path('frasier_gpsr')
yaml_dirname = pkg_dir + '/config/'

# Python
import os
import datetime
import yaml
import random

QUESTION_TO_ANSWER = yaml.load(open(yaml_dirname + '/question2answer.yaml'))
PROMPT_TO_ANSWER = yaml.load(open(yaml_dirname + '/prompt2answer.yaml'))
JOKES = [line.replace('\n', '') for line in open(yaml_dirname + '/jokes.txt').readlines()]


def levenshtein_distance(s1, s2):
    if len(s1) < len(s2):
        return levenshtein_distance(s2, s1)

    # len(s1) >= len(s2)
    if len(s2) == 0:
        return len(s1)

    previous_row = range(len(s2) + 1)
    for i, c1 in enumerate(s1):
        current_row = [i + 1]
        for j, c2 in enumerate(s2):
            insertions = previous_row[j + 1] + 1  # j+1 instead of j since previous_row and current_row$
            deletions = current_row[j] + 1  # than s2
            substitutions = previous_row[j] + (c1 != c2)
            current_row.append(min(insertions, deletions, substitutions))
        previous_row = current_row

    return previous_row[-1]


def get_closest_list_member(s0, list_of_s):
    min_distance = None
    for s in list_of_s:
        distance = levenshtein_distance(s0, s)
        if min_distance is None or min_distance > distance:
            min_distance = distance
            best_match = s
    return best_match


def get_answer_to_closest_question(question, dictionary=QUESTION_TO_ANSWER):
    min_distance = None
    best_match = get_closest_list_member(question, dictionary.keys())
    answer = dictionary[best_match]
    return answer


def get_answer_to_closest_prompt(prompt):
    answer = get_answer_to_closest_question(prompt, dictionary=PROMPT_TO_ANSWER)

    if answer == 'TIME':
        minute = str(datetime.datetime.now().time().minute)
        hour = str(datetime.datetime.now().time().hour)
        return 'The time is ' + hour + ' ' + minute
    elif answer == 'TOMORROW':
        tomorrow = datetime.datetime.now() + datetime.timedelta(days=1)
        tomorrow_weekday = tomorrow.strftime('%A')
        tomorrow_day = tomorrow.strftime('%m')
        return 'Tomorrow is ' + tomorrow_weekday + ' the ' + tomorrow_day
    elif answer == 'WEEKDAY':
        weekday = datetime.datetime.now().strftime('%A')
        return 'Today is ' + weekday
    elif answer == 'DAY':
        day = datetime.datetime.now().strftime('%m')
        return 'Today is the ' + day
    elif answer == 'JOKE':
        index = random.randint(0, len(JOKES) - 1)
        return JOKES[index]
    else:
        return answer

