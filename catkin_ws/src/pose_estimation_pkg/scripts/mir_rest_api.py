#! /usr/bin/env python

import rospy
import std_msgs
import requests, json, os
import datetime
import time

ip = '192.168.12.20'
host = 'http://' + ip + '/api/v2.0.0/'
#Format Headers
headers = {}
headers['Content-Type'] = 'application/json'
headers['Authorization'] = 'Basic Um9ib3RMYWI6MTM5OTA5MzdhYjhjYTQ0MTM3NTFhOTAxMmEzMTI1NWU3MmExOGJhM2FjOGQ1YzgzZGQ1MTFkZjUwY2YyYTNlMQ=='

# GET basic info, battery, status, etc
def get_status(host):
    url = 'status'
    get_request = requests.get(host + url)
    status = get_request.status_code
    if status == 200: #if there is an established connection
        txt = get_request.text
        status_dict = json.loads(txt)
        state = str(status_dict.get('state_text'))
        if state == 'Ready':
            return False
        else:
            return True

# GET mission names 
def get_missions(host,headers):
    url = 'missions'
    get_request = requests.get(host + url, headers = headers)
    txt = get_request.text
    status_dict = json.loads(txt)
    print(status_dict)

# POST a known mission
def post_mission(host,data,headers):
    url = 'mission_queue'
    response = requests.post(host + url, json = data, headers = headers)

# Execute a mission
def execute_mission(host,mission,headers):
    print("Executing mission...")
    post_mission(host,mission,headers)
    executing = True
    time.sleep(2)
    while executing == True:
        executing = get_status(host)
    print("Mission executed!")


if __name__ == "__main__":

    rospy.init_node("mir_rest_api_node")
    pub_mir_status = rospy.Publisher("/navigation/mir_status", std_msgs.msg.String, queue_size=1)

    # Display stored missions
    #get_missions(host,headers)

    Charge_mission = {"mission_id": "8e1de7fe-32e5-11ea-89a4-000129922f1c"} #GUID for "Charge" mission
    dock_assembly_line_mission = {"mission_id": "6901204f-c200-11eb-8133-000129922f1c"} #GUID for "dock_assembly_line" mission
    Go_to_start_point_mission = {"mission_id": "0601374b-cd13-11eb-8133-000129922f1c"} #GUID for "Go_to_start_point" mission
    Go_to_entry_point_mission = {"mission_id": "fdc728b0-c21b-11eb-8133-000129922f1c"} #GUID for "Go_to_entry_point" mission

    time_start = time.time()
    # Execute missions
    #execute_mission(host, Go_to_entry_point_mission, headers)
    execute_mission(host, Go_to_start_point_mission, headers)
    execute_mission(host, dock_assembly_line_mission, headers)
    time.sleep(5.0)
    time_end = time.time()
    print(":: MIR Mission executing time: %.3f seconds" %(time_end-time_start))
    pub_mir_status.publish("   Robot docked to the assembly line. Ready to feed the fixture!")
    feeding_task_flag = rospy.wait_for_message("/pose_estimation/status", std_msgs.msg.String, rospy.Duration(8000.0))

    if feeding_task_flag.data == "feeding_task_done":
        execute_mission(host, Go_to_start_point_mission, headers)
        print(":: Assembly line fed!")

    rospy.spin()
    quit()