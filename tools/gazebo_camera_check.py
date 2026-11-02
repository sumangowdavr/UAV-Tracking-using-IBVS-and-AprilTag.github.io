"""Test AprilTag image detection and XY control against a live Gazebo camera.

This is an isolated camera-rig bench, not a PX4/multirotor flight test. The
actual Gazebo renderer supplies images; OpenCV detects a 36h11 tag; this repo's
compute_xy_velocity controls a kinematically positioned camera rig.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cv2
import numpy as np


def make_world(path):
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    bits=cv2.aruco.drawMarker(dictionary,0,8)
    visuals=['<visual name="white_board"><geometry><box><size>1 1 .01</size></box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>']
    for row in range(8):
        for col in range(8):
            if bits[row,col]==0:
                visuals.append(f'<visual name="cell_{row}_{col}"><pose>{(col-3.5)*.1} {(3.5-row)*.1} .006 0 0 0</pose><geometry><box><size>.1001 .1001 .002</size></box></geometry><material><ambient>0 0 0 1</ambient><diffuse>0 0 0 1</diffuse></material></visual>')
    world=f'''<?xml version="1.0"?><sdf version="1.8"><world name="apriltag_camera_bench">
    <physics name="physics" type="ignored"><max_step_size>.001</max_step_size><real_time_factor>1</real_time_factor></physics>
    <plugin filename="libignition-gazebo-physics-system.so" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="libignition-gazebo-user-commands-system.so" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="libignition-gazebo-scene-broadcaster-system.so" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="libignition-gazebo-sensors-system.so" name="ignition::gazebo::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>1 1 1 1</ambient><background>.55 .62 .68 1</background></scene>
    <light name="sun" type="directional"><pose>0 0 8 0 0 0</pose><diffuse>.8 .8 .8 1</diffuse><direction>0 0 -1</direction><cast_shadows>false</cast_shadows></light>
    <model name="floor"><static>true</static><pose>0 0 -.04 0 0 0</pose><link name="link"><visual name="floor"><geometry><box><size>8 8 .05</size></box></geometry><material><ambient>.5 .56 .6 1</ambient><diffuse>.5 .56 .6 1</diffuse></material></visual></link></model>
    <model name="tag_36h11_id0"><static>true</static><link name="tag">{''.join(visuals)}</link></model>
    <model name="camera_rig"><static>true</static><pose>.45 .25 2.3 0 1.57079632679 0</pose><link name="camera">
    <sensor name="downward" type="camera"><always_on>true</always_on><update_rate>15</update_rate><topic>/apriltag_bench/image</topic><camera><horizontal_fov>1.15</horizontal_fov><image><width>960</width><height>600</height><format>R8G8B8</format></image><clip><near>.02</near><far>20</far></clip></camera></sensor></link></model>
    </world></sdf>'''
    path.write_text(world)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True,type=Path)
    args=parser.parse_args();out=args.output_dir.resolve();out.mkdir(parents=True,exist_ok=True)
    frames=out/'frames';frames.mkdir(exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    package=root/'ros_uav_apriltag';sys.path.insert(0,str(package))
    from ros_uav_apriltag.control_utils import compute_xy_velocity
    env=os.environ.copy();env.update({'ROS_DOMAIN_ID':'97','ROS_LOCALHOST_ONLY':'1','IGN_PARTITION':'apriltag_camera_'+str(os.getpid()),'ROS_LOG_DIR':str(out/'ros_logs')})
    os.environ.update({k:env[k] for k in ['ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','IGN_PARTITION','ROS_LOG_DIR']})
    world=out/'camera_bench.sdf';make_world(world)
    processes=[];handles=[]
    def start(cmd,name):
        f=(out/(name+'.log')).open('w');handles.append(f)
        p=subprocess.Popen(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p)
    try:
        start(['ign','gazebo','-r','-s','--headless-rendering',str(world)],'gazebo')
        start(['/opt/ros/humble/lib/ros_gz_bridge/parameter_bridge',
               '/apriltag_bench/image@sensor_msgs/msg/Image[ignition.msgs.Image',
               '/world/apriltag_camera_bench/set_pose@ros_gz_interfaces/srv/SetEntityPose'],'bridge')
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from ros_gz_interfaces.srv import SetEntityPose
        from ros_gz_interfaces.msg import Entity
        rclpy.init();node=Node('apriltag_camera_check')
        latest={}
        def image_cb(msg):latest['image']=msg
        node.create_subscription(Image,'/apriltag_bench/image',image_cb,qos_profile_sensor_data)
        client=node.create_client(SetEntityPose,'/world/apriltag_camera_bench/set_pose')
        assert client.wait_for_service(timeout_sec=25),'Gazebo set-pose service missing'
        dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params=cv2.aruco.DetectorParameters_create()
        f=960/(2*math.tan(1.15/2));k=np.array([[f,0,480],[0,f,300],[0,0,1]],dtype=float)
        rows=[];unique_frames=0;last_stamp=None;start_time=time.monotonic();x,y=.45,.25;consecutive=0;initial_error=None
        while time.monotonic()-start_time<60:
            rclpy.spin_once(node,timeout_sec=.1)
            if 'image' not in latest:continue
            msg=latest['image'];stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
            if stamp==last_stamp:continue
            last_stamp=stamp
            image=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
            bgr=cv2.cvtColor(image,cv2.COLOR_RGB2BGR) if msg.encoding=='rgb8' else image.copy()
            corners,ids,rejected=cv2.aruco.detectMarkers(bgr,dictionary,parameters=params)
            found=ids is not None and 0 in ids.ravel()
            err=float('nan');vx=vy=0.
            if found:
                chosen=int(np.where(ids.ravel()==0)[0][0]);selected=[corners[chosen]]
                rvecs,tvecs,_=cv2.aruco.estimatePoseSingleMarkers(selected,.8,k,np.zeros(5))
                t=tvecs[0,0];ex,ey=-float(t[1]),-float(t[0]);err=math.hypot(ex,ey)
                if initial_error is None:initial_error=err
                vx,vy=compute_xy_velocity(ex,ey,.7,.8)
                cv2.aruco.drawDetectedMarkers(bgr,selected,np.array([[0]],dtype=np.int32))
                cv2.drawFrameAxes(bgr,k,np.zeros(5),rvecs[0],tvecs[0],.18)
                consecutive=consecutive+1 if err<.06 else 0
                x+=vx*.1;y+=vy*.1
                request=SetEntityPose.Request();request.entity.name='camera_rig';request.entity.type=Entity.MODEL
                request.pose.position.x=x;request.pose.position.y=y;request.pose.position.z=2.3
                request.pose.orientation.y=math.sin(math.pi/4);request.pose.orientation.w=math.cos(math.pi/4)
                future=client.call_async(request)
                rclpy.spin_until_future_complete(node,future,timeout_sec=3)
                assert future.done() and future.result().success,'Camera-rig pose command failed'
            cv2.line(bgr,(470,300),(490,300),(0,180,255),1);cv2.line(bgr,(480,290),(480,310),(0,180,255),1)
            cv2.rectangle(bgr,(0,0),(960,46),(30,30,30),-1)
            label=f'Gazebo camera | AprilTag 36h11 ID 0 | XY offset {err:.3f} m' if found else 'Gazebo camera | searching for AprilTag 36h11 ID 0'
            cv2.putText(bgr,label,(15,29),cv2.FONT_HERSHEY_SIMPLEX,.65,(240,240,240),1,cv2.LINE_AA)
            filename=f'{unique_frames:05d}.jpg';cv2.imwrite(str(frames/filename),bgr,[cv2.IMWRITE_JPEG_QUALITY,94])
            rows.append({'frame':unique_frames,'stamp_s':stamp,'tag_detected':bool(found),'offset_m':err,'vx':vx,'vy':vy,'rig_x':x,'rig_y':y})
            unique_frames+=1
            if consecutive>=8:break
        node.destroy_node();rclpy.shutdown()
        assert consecutive>=8,'Tag centering did not converge'
        assert rows[-1]['offset_m']<.06 and initial_error>.3
        result={'status':'PASS','frames':len(rows),'detected_frames':sum(r['tag_detected'] for r in rows),'initial_offset_m':initial_error,'final_offset_m':rows[-1]['offset_m'],'family':'AprilTag 36h11','id':0,'renderer':'Gazebo Fortress camera','scope':'OpenCV tag detection + public XY utility; kinematic camera rig; no PX4, VIO, mission manager or flight dynamics'}
        with (out/'trace.csv').open('w') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        subprocess.run(['ffmpeg','-hide_banner','-loglevel','error','-y','-framerate','10','-i',str(frames/'%05d.jpg'),'-vf','fps=20','-an','-c:v','libx264','-crf','21','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'gazebo-apriltag-camera.mp4')],check=True)
        print(json.dumps(result,indent=2),flush=True)
    finally:
        for p in reversed(processes):
            if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
        for p in processes:
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for f in handles:f.close()


if __name__=='__main__':main()
