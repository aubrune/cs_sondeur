#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from std_msgs.msg import String

FILE_PATH = "/home/arbalet/Documents/USB_KEY_DEMO/"
VOTE_NAME = "fear"
NB_MAX_BLOCKS = 2       #Nombre maximum de blocs sur chaque ligne
BUTTON_OVERLOAD = 2

time_previous_vote = 0  #Temps noté au vote précédent
nb_total_votes = 0

def save_vote():
    print "Sauvegarde du vote"
    file_name = FILE_PATH + VOTE_NAME + "_" + time.strftime("%Y-%B-%d_%H-%M-%S") + ".json"
    file_content = '{"'+VOTE_NAME+'":"'+time.strftime("%Y/%B/%d %H:%M:%S")+'"}'
    vote_file = open(file_name, "w")
    vote_file.write(file_content)
    vote_file.close()
    
if __name__=='__main__':        #MAIN
    rospy.init_node('edo_gpio') #Initialisation du node 'edo_movement'
    publisher = rospy.Publisher('vote', String, queue_size=10)
    rospy.sleep(1)                           
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        vote = raw_input("Appuyez sur entrée pour voter ou q pour quitter : ")
        if vote == "q":
            break
        if (time.time() - time_previous_vote) <= BUTTON_OVERLOAD:
            print "Trop peu de temps entre chaque vote"
        else:
            print "Vote pris en compte"
            time_previous_vote = time.time()
            nb_total_votes += 1
            #nb_votes_block += 1
            rospy.loginfo(nb_total_votes)
            publisher.publish(str(nb_total_votes))
            #print "Nombre de votes total :", nb_total_votes
            #print "Nombre de votes dans le bloc courant :", nb_votes_block
            #save_vote()
            #if ((nb_total_votes - 1) % 5) == 0 and nb_total_votes != 1:
                #if nb_total_blocks % NB_MAX_BLOCKS == 0:
                    #print "change_line()"
                #else:
                    #print "change_block()"
                    #nb_total_blocks += 1
                #print "trace_stick()"
            #elif nb_total_votes % 5 == 0:
                #print "trace_line()"
                #nb_votes_block = 0
            #else:
                #if nb_votes_block != 1:
                    #print "move_to_next_stick()"
                #print "trace_stick()"
    
