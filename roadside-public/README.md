1. cd \<your catkin workspace\>/src  
2. git clone https://gitlab.mff.cuni.cz/obdrd3am/roadside-public.git  
3. cd roadside-public

4. chmod +x set_variables.sh

5. modify the script set_variables.sh if you are using a resource directory other than /usr/share/gazebo-11

6. ./set_variables.sh

7.  open new tab/window since change of enviromental variables take effect only with with new tab/window

basic test world is working with:

8. cd worlds

9. gazebo --verbose road_assistance.world

alternatively you can use test world with game elements:

9. gazebo --verbose road_assistance_with_objects.world

