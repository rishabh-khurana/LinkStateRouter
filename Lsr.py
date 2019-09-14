#!/usr/bin/python3
'''COMP9331 
Assignment 
By Rishabh Khurana 
zID-z5220427 '''

import sys
from socket import *
import threading
import time
import logging
import math

# wait for one second to re-transmit link state packet
UPDATE_INTERVAL=1

# wait for 30 seconds after boot up to calculate least cost 
ROUTE_UPDATE_INTERVAL=30

file_data=[]
try:
    file_name=sys.argv[1]
    with open(file_name) as file1:
        for line in file1:
            file_data.append(line.strip().split())
except IndexError:
    print("file name not specified")
    exit()
except FileNotFoundError:
    print("file not found")
    exit()

# compare two dictionaries
def same_vals(message_broadcasted,neighbour_seq):
    FLAG=False
    for n in neighbour_seq:
        # if any value is same as before return True
        if neighbour_seq[n]==message_broadcasted[n]:
            FLAG=True
            return FLAG
        else:
            continue
    return FLAG
        


# In case of node failure update neigbours of router
def node_failure(failed_node):
    global neighbours_data
    index=0
    for i in range(len(neighbours_data)):
        if neighbours_data[i][0]==failed_node:
            index=i
    del neighbours_data[index]

# extract data from file
router_data=file_data[:2]
neighbours_data=file_data[2:]
neighbours_set=neighbours_data
failed_nodes=[]
failed_topo=[]

# assign information to variables
router_ID=router_data[0][0]
router_PORT=int(router_data[0][1])

#initialize UDP socket
s=socket(AF_INET,SOCK_DGRAM)
s.setsockopt(SOL_SOCKET,SO_REUSEADDR,1)
s.bind(("localhost",router_PORT))

# initialize the topology as neighbours
topology={}

topology[router_ID]={}

for neighbour in neighbours_data:
    topology[router_ID][neighbour[0]]=float(neighbour[1])

def calc_start_point(path):
    topo=topology
    dist=0
    for i in range(len(path)):
        if i+1<len(path):
            if path[i] in topo.keys():
                dist=dist+float(topo[path[i]][path[i+1]])
    return dist

#Dijkstra's Algorithm 
def calc_least_cost():
    # hello=0
    while True:
        time.sleep(ROUTE_UPDATE_INTERVAL)
        my_neighbours=neighbours_data
        topo=topology
        # nodes that have been visited
        shortest_dist={}
        # dict of nodes with distances and previous node
        current_table={}
        # add neighbours to current table 
        for n in my_neighbours:
            current_table[n[0]]={}
            current_table[n[0]]['dist']=float(n[1])
            current_table[n[0]]['prev']=router_ID
        # initialize other nodes besides neighbours
        for t in topo:
            if t not in [n[0] for n in my_neighbours] and not (t==router_ID):
                current_table[t]={}
                current_table[t]['dist']=float(math.inf)
                current_table[t]['prev']=router_ID

        # select the neighbour with the minimum distance
        while(len(current_table)):
            for t in current_table:
                if current_table[t]['dist']==min(current_table[x]['dist'] for x in current_table):
                    curr_neighbour=t
                # add to distance list
                    shortest_dist[t]={}
                    shortest_dist[t]['dist']=current_table[t]['dist']
                    shortest_dist[t]['path']=current_table[t]['prev']+t
            # update other distances in the current table
            start_point=calc_start_point(shortest_dist[curr_neighbour]['path'])
            #start_point=float(topo[router_ID][curr_neighbour])
            for i in topo[curr_neighbour]:
                if i in current_table:
                    if float(topo[i][curr_neighbour])+start_point<current_table[i]['dist']:
                        current_table[i]['dist']=start_point+float(topo[i][curr_neighbour])
                        current_table[i]['prev']=shortest_dist[curr_neighbour]['path']
            # remove node with minimum distance
            del current_table[curr_neighbour]
        print(f'I am router {router_ID}')
        for i in shortest_dist:
            print(f"Least cost to router {i}:{shortest_dist[i]['path']} and the cost is {round(shortest_dist[i]['dist'],1)}")

def recv_messages():
    global failed_topo
    global failed_nodes
    global topology
    # list of message ids that have already been broadcasted
    message_broadcasted={}
    timer_started=False
    neighbour_seq={}
    start_time=0
    while True:
        my_neighbours=neighbours_data
        message,_=s.recvfrom(router_PORT)
        message=message.decode('utf-8')
        message_data=message.strip().split()
        if message_data[1] in message_broadcasted:
            if not (message_broadcasted[message_data[1]]==message_data[0]):
                if int(len(message_data[3:])/2)<len(topology[message_data[1]]):
                    for i in topology[message_data[1]]:
                        if i not in message_data[3:]:
                            if (i not in failed_topo) and (i not in failed_nodes):
                                if i not in [n[0] for n in my_neighbours]:
                                    failed_topo.append(i)
                                    del topology[i]
                                else:
                                    failed_nodes.append(i)
                                    del topology[i]
                                    node_failure(i)

        # add info to topology table
        topology[message_data[1]]={}
        temp=message_data[3:]
        for i in range(len(temp)):
            if i%2==0:
                topology[message_data[1]][temp[i]]=temp[i+1]
        for neighbour in my_neighbours:
            # do not send neighbour's heart beat back to the neigbour node
            if not(neighbour[0] is message_data[1]):
                # supress messages which have already been broadcasted
                if (message_data[1] not in message_broadcasted): 
                    new_s=socket(AF_INET,SOCK_DGRAM)
                    new_s.sendto(message.encode('utf-8'),("localhost",int(neighbour[2])))
                    new_s.close()
                else:
                    if not (message_broadcasted[message_data[1]]==message_data[0]):
                        new_s=socket(AF_INET,SOCK_DGRAM)
                        new_s.sendto(message.encode('utf-8'),("localhost",int(neighbour[2])))
                        new_s.close()
        # update sequence no for a specific router
        if (message_data[1] not in message_broadcasted):
            message_broadcasted[message_data[1]]=''
            message_broadcasted[message_data[1]]=message_data[0]
        else:
            if not (message_broadcasted[message_data[1]]==message_data[0]):
                message_broadcasted[message_data[1]]=message_data[0]
                # update topology if different
                
        # neigbour node failure detection
        # start timer
        if timer_started==False:
            # note down all SEQ values of active neighbours
            for m in message_broadcasted:
                if m in [n[0] for n in my_neighbours]:
                    neighbour_seq[m]=message_broadcasted[m]
            # start timer
            timer_started=True
            start_time=time.time()
        else:
            if int(time.time()-start_time) < 3:
                # if all SEQ vals have changed
                if same_vals(message_broadcasted,neighbour_seq)==False:
                    # restart timer
                    start_time=time.time()
                    neighbour_seq={}
                    # note down all SEQ values of active neighbours
                    for m in message_broadcasted:
                        if m in [n[0] for n in my_neighbours]:
                            neighbour_seq[m]=message_broadcasted[m]

            if int(time.time()-start_time) > 3:
                # SEQ vals of any node remains same
                if same_vals(message_broadcasted,neighbour_seq)==True:
                # node failed
                    for n in neighbour_seq:
                        if neighbour_seq[n]==message_broadcasted[n] and (n not in failed_nodes):
                            failed_nodes.append(n)
                            del topology[n]
                            node_failure(n)
                            # stop timer
                            timer_started=False
                            start_time=0
        

# send link state packet after every second to all neighbours
def send_heart_beat():
    SEQ_NO=0
    while True:
        my_neighbours=neighbours_set
        #set Link State Packet Data
        Link_state=str(SEQ_NO)+' '+router_ID+' '+str(router_PORT)

        for n in my_neighbours:
            if (n[0] not in failed_nodes):
                Link_state=Link_state+' '+n[0]+' '+n[1]
        for neighbour in my_neighbours:
            # set up UDP socket
            s=socket(AF_INET,SOCK_DGRAM)
            s.sendto(Link_state.encode('utf-8'),("localhost",int(neighbour[2])))
            s.close()
        # sleep for 1 second
        SEQ_NO=SEQ_NO+1
        time.sleep(UPDATE_INTERVAL)

#Start three threads
try:
    #recv message thread
    recv_msg=threading.Thread(target=recv_messages)
    recv_msg.start()
    #heart beat thread
    heart_beat=threading.Thread(target=send_heart_beat)
    heart_beat.start()
    #calculation thread
    calc_dist=threading.Thread(target=calc_least_cost)
    calc_dist.start()
except KeyboardInterrupt:
    recv_messages.kill_received=True
    heart_beat.kill_received=True
    calc_dist.kill_received=True
    s.close()
    sys.exit()