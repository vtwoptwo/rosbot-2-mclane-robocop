import socket
import streamlit as st
import time
import logging
import sys
import warnings
warnings.filterwarnings("ignore")
import ast
# Path to your video file
video_file_beg = 'bruce_willis_mayday.mp4'
video_file_bad = 'freshy_fruits.mp4'
good_guy = 'yes-the-final-scene.jpg'

UDP_IP = '10.205.3.76'
UDP_PORT = 6667

st.header('Mclane Robocop')
logging.info('run')


#st hidden toggle
with st.expander('Slides'):
    st.write('How does MCLANE work?')
    st.image('1.jpg')
    st.image('2.jpg')
    st.image('3.jpg')
    st.image('4.jpg')
    st.image('5.jpg')
    st.image('6.jpg')
    st.image('7.jpg')


    


@st.cache()
def bind_socket(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Change to UDP
    sock.bind((ip, port))
    return sock

def main():
    sock = bind_socket(UDP_IP, UDP_PORT)
    global RUN
    RUN = True
    st.video(video_file_beg)
    while RUN:
        data, addr = sock.recvfrom(1024)  # Receive data from the socket
        if not data:
            break  # If no data, leave the loop
        # Decode data
        data = data.decode()
        print(data)
        
        if data: 
            print("DATA: ", data)
            probs, classes = data.split('bam')
            # do a literal eval of probs and classes
            probs = float(probs)
            classes = float(classes)
            st.write(f"Probs: {probs}")
            st.write(f"Classes: {classes}")
            # logic -> 
            # dictionary of classes to bad or good  change classes depending on what 
            rules = {0:0,
                        1:0,
                        2:0,
                        3:0,
                        4:0,
                        5:0,
                        6:1, 
                        7:0,
                        8:0,
                        9:0,
                        10:0,
                        11:0,
                        12:0,
                        13:0,
                        14:0}
            if classes in rules.keys():
                signal_new = rules[classes] 

                if signal_new == 1:
                    st.write('Bad')
                    st.video(video_file_bad)
                elif signal_new == 0:
                    st.write('Good')
                    st.image(good_guy)

                else: 
                    st.write('Scouting for bad guys')
                    st.image('scouting.jpg')
        time.sleep(1)

    #

   
                            

                    
    

if __name__ == "__main__":
    main()
