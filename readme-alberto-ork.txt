================================================================================================================
ORK Linemod object detection module
================================================================================================================

- Delete object
  $ cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_delete.py e9a4f4c2f2308cd9968365268f000a5d --commit

- Creating objects
cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n "expo_dry_erase_board_eraser" -d "An white board eraser." --commit
cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n "highland_6539_self_stick_notes" -d "Stick notes." --commit
cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n "crayola_64_ct" -d "Crayola pencils." --commit
cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n "champion_copper_plus_spark_plug" -d "A spark plug." --commit
cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n "cylinder_red" -d "A red cylinder." --commit
cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n "cylinder_green" -d "A green cylinder." --commit
cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core object_add.py -n "cylinder_blue" -d "A blue cylinder." --commit

- Adding object mesh
  $ cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core mesh_add.py e9a4f4c2f2308cd9968365268f0760cc expo_dry_erase_board_eraser.obj --commit
  
- Training
  $ cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core training -c `rospack find object_recognition_linemod`/conf/training.ork
  
- Testing
  Open 5 terminal and run in all of then 
  $ source devel/setup.sh
  Run the following commands, one on each terminal
  $ roscore
  $ roslaunch freenect_launch freenect.launch
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun tf static_transform_publisher 0 0 0 0 0 0 camera_rgb_optical_frame mapping_camera_frame 10
  $ rosrun rviz rviz --display-config src/linemod/conf/linemod_test.rviz
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork



================================================================================================================
ORK VG-RAM object detection module
================================================================================================================

- Before compiling
  $ sudo mv /opt/ros/hydro/include/opencv2/objdetect/objdetect.hpp /opt/ros/hydro/include/opencv2/objdetect/objdetect.hpp.old
  $ sudo cp apc/code/opencv-2.4.10/modules/objdetect/include/opencv2/objdetect/objdetect.hpp /opt/ros/hydro/include/opencv2/objdetect/objdetect.hpp
  
- Training
  $ cd $PRACSYS_PATH/../object_models; rosrun object_recognition_core training -c `rospack find object_recognition_vg_ram`/conf/training.ork
  
- Testing
  Open 5 terminal and run in all of then 
  $ source devel/setup.sh
  Run the following commands, one on each terminal
  $ roscore
  $ roslaunch freenect_launch freenect.launch
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun rviz rviz --display-config src/vg_ram/conf/vg_ram_test.rviz
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_vg_ram`/conf/detection.ros.ork


  
================================================================================================================
ORK model handling
================================================================================================================

- Documentation
  http://wg-perception.github.io/object_recognition_core/index.html

- Create a plane template
  $ rosrun object_recognition_capture orb_template -o my_textured_plane
  
- Track (test) a plane template
  $ rosrun object_recognition_capture orb_track --track_directory my_textured_plane

- Capture a bag ((i) tirar o -i para usar o plane taemplate em xxxx; (ii) tirar preview para de fato capturar)
  $ rosrun object_recognition_capture capture -i my_textured_plane --seg_z_min 0.01 -o expo_dry_erase_board_eraser.bag --preview

- Uploading a bag to the database (TOD)
  $ rosrun object_recognition_capture upload -a 'Alberto F. De Souza' -e 'alberto@lcad.inf.ufes.br' -i expo_dry_erase_board_eraser.bag -n 'expo_dry_erase_board_eraser' -d 'An white board eraser.' eraser, amazon, tod --commit
  
- Build a mesh from a bag and add it to the database
  $ rosrun object_recognition_reconstruction mesh_object --all --visualize --commit

- Delete object
  $ rosrun object_recognition_core object_delete.py e9a4f4c2f2308cd9968365268f000a5d --commit

  
- Database 
  view all: http://127.0.0.1:5984/_utils/database.html?object_recognition/_design/objects/_view/all
  meshes : http://localhost:5984/or_web_ui/_design/viewer/meshes.html



================================================================================================================
ORK Tabletop object detection module
================================================================================================================

- Training
  Not necessary
  
- Detection
  Open 5 terminal and run in all of then 
  $ source devel/setup.sh
  Run the following commands, one on each terminal
  $ roscore
  $ roslaunch openni_launch openni.launch
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosrun rviz rviz
  $ rosrun object_recognition_core detection -c  `rospack find object_recognition_tabletop`/conf/detection.object.ros.ork



================================================================================================================
ORK TOD object detection module
================================================================================================================

- Training (all objects in the database)
  $ rosrun object_recognition_core training -c `rospack find object_recognition_tod`/conf/training.ork --visualize --commit
  
- Testing
  $ rosrun object_recognition_core detection -c `rospack find object_recognition_tod`/conf/detection.ros.ork --visualize



================================================================================================================
Debuging
================================================================================================================

$ cd object_models
{see http://pymotw.com/2/pdb/}
$ python -m pdb ../src/object_recognition_core/apps/training -c ../src/vg_ram/conf/training.ork
$ gdb --args /usr/bin/python ../src/object_recognition_core/apps/training -c `rospack find object_recognition_vg_ram`/conf/training.ork
$ gdb --args /usr/bin/python ../src/object_recognition_core/apps/detection -c `rospack find object_recognition_vg_ram`/conf/detection.ros.ork

$ cd object_models
$ gdb --args /usr/bin/python ../src/object_recognition_core/apps/training -c `rospack find object_recognition_linemod`/conf/training.ork
$ gdb --args /usr/bin/python ../src/object_recognition_core/apps/detection -c `rospack find object_recognition_linemod`/conf/detection.ros.ork

In ORK:
break ecto_linemod::Detector::process

In OpenCV:
break cv::linemod::Detector::matchClass



================================================================================================================
Ground truth generation
================================================================================================================

- Save a list of objects to generate ground truth
  $ cd object_models; ls data_gathered/ | grep depth | sort -r > data_gathered.txt; cd ..

- Run linemod and auxiliary programs. You will need a kinect to pump the pipeline, but its images will not be used.
  $ roscore
  $ roslaunch freenect_launch freenect.launch
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosparam set /ground_truth_generation true
  $ rosrun rviz rviz --display-config src/linemod/conf/data_gathering.rviz
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork

- Check the user interface in the code: src/linemod/src/linemod_detect.cpp {ground_truth_generation() function}



================================================================================================================
Object detection system evaluation
================================================================================================================

- Save a list of objects to evaluate
  $ cd object_models; ls data_gathered/ | grep depth > data_gathered.txt; cd ..

- Run linemod and auxiliary programs. You will need a kinect to pump the pipeline, but its images will not be used.
  $ roscore
  $ roslaunch freenect_launch freenect.launch
  $ rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True
  $ rosparam set /object_detection_evaluation true
  $ rosrun rviz rviz --display-config src/linemod/conf/data_gathering.rviz
  $ cd $PRACSYS_PATH/../object_models; export ROS_HOME=.; rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork

- The results will be saved into $PRACSYS_PATH/../object_models/object_detection_performance.txt



================================================================================================================
TODO
================================================================================================================

Alterar vg_ram para salvar um dado binario de teste (vetor de char? de int?) no db no train e ler no test
Implementar vg_ram com parametros hard coded. Usar a facility acima para salvar o aprendizado. Manter linemod funcionando ate o fim (via _old nas funcoes)
Mudar os parametros para parametros proprios de vg_ram 

Tem que voltar o arquivo vg_ram.cpp para o opencv e eliminar estas copias:
  $ cp ./modules/objdetect/src/precomp.hpp ~/RUTGERS/apc_hg/src/vg_ram/src/
  $ cp cvconfig.h ~/RUTGERS/apc_hg/src/vg_ram/src/
  $ cp ./modules/objdetect/src/normal_lut.i ~/RUTGERS/apc_hg/src/vg_ram/src/

Mover o arquivo vg_ram.cpp de volta para o OpenCV quando acabar a APC

src/vg_ram/sample/detection.py:        pub_rgb = ecto_ros.ecto_sensor_msgs.Publisher_Image("image pub", topic_name='linemod_image')

Testar features de cor no meio do objeto para o cone e o eraser
Resolver problema de iluminacao
Checar o computo da similaridade (threshold) de aceitacao - por que um percentual tao alto?

Parece que o linemod esta compondo as features de cor e depth errado, porque detecta o objecto em posicoes muito diferentes da do treino 
 com threshold 75
Tem que checar o linearize 
Tem que checar o computo da pose do objeto, i.e. a composicao das features em um ponto (sera que faz igual ao paper da Mariella?)

Por que as features estao aparecendo deslocadas para a esquerda (as setas?)?
O problema parece ser no calculo do angulo das features de cor

Ver porque esta repetindo of Info

Examine freenect package:

sudo apt-get install ros-hydro-freenect-stack

roslaunch freenect_launch freenect.launch

rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True

rosrun dynamic_reconfigure dynparam set /camera/driver image_mode 1

rosrun dynamic_reconfigure dynparam set /camera/driver depth_mode 1

rosrun topic_tools relay /camera/depth_registered/image_raw /camera/depth_registered/image

