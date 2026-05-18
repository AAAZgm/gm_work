#不用手势
cp -r /opt/tros/humble/lib/mono2d_body_detection/config /home/sunrise/gm_car/colcon_ws
#用手势
cp -r /opt/tros/${TROS_DISTRO}/lib/mono2d_body_detection/config/ /home/sunrise/gm_car/colcon_ws
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_lmk_detection/config/ /home/sunrise/gm_car/colcon_ws
cp -r /opt/tros/${TROS_DISTRO}/lib/hand_gesture_detection/config/ /home/sunrise/gm_car/colcon_ws
cp -r /opt/tros/${TROS_DISTRO}/lib/hobot_audio/config/ /home/sunrise/gm_car/colcon_ws