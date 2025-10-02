Main.java
==========
Add RobotSpec_<> constructors to Main.java.  Each must have the serialnum for the
RoboRio used by that machine. 

Setting serialnum for debug
===========================

In the VSCode's Debug:Main power shell, used for starting the simulationm, needs to 
have the serialnum set to select desired robot spec to use in debug.

In the Powershell:
  $env:serialnum ='031b7511'          #swerveBot aka Tim
  $env:serialnum ='032D2062'          #compbot 2024
  $env:serialnum ='03238151'          #Chadbot
  $env:serialnum ='032381BF'          #alphabot 2025     
  $env:serialnum ='03061025'          #botOnBoard2
Pick one of these before debugging in simulation mode.

You can show env with:
Get-ChildItem Env:serialnum  


GIT SubModule Commands
git submodule status --recursive
git submodule update --recursive --remote
