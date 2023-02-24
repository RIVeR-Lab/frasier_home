# ROS
import rospkg

pkg_dir = rospkg.RosPack().get_path('frasier_gpsr')
yaml_dirname = pkg_dir + '/config/'

# Python
import os
import datetime
import yaml

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
    if float(distance) / float(max(len(best_match),len(s0)))>0.3:
        return ''
    return best_match
