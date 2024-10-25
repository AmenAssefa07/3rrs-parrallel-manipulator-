# 3rrs-parrallel-manipulator-
Bu'uura - ball ballancing robot

ball_tracker.py - camera node, takes camera feed and publishes pos and vel 
motor_control_node.py - subscribes to desired state (pitch,roll,height) and sends pwm signals over uart to control servo motors
