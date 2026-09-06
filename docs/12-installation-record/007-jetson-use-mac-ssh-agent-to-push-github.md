# Flow
```
MacBook (Have GitHub SSH key)
        │
        │ ssh -A
        ▼
Jetson (No key)
        │
        ▼
GitHub
```

# Mac config
## mark sure macbook key can access github
```
ssh -T git@github.com
```
### if succ you will see below message
```
Hi <team-github-account>! You've successfully authenticated, but GitHub does not provide shell access.
```
## confirm key in ssh-agent
```
ssh-add -l
```
```
<team-user>@<jetson-host> ~ % ssh-add -l
The agent has no identities.
```
### if key is not in ssh-agent, added
```
ssh-add ~/.ssh/id_rsa
```
```
<team-user>@<jetson-host> ~ % ssh-add ~/.ssh/id_rsa
Identity added: $HOME/.ssh/id_rsa (lily_wang1983000@cs-553581357317-default)
```
# mac login jetson
**-A means Agent Forwarding**
```
ssh -A <team-user>@<jetson-host>
```
## check jetson env setting
```
echo $SSH_AUTH_SOCK
```
### if you see below, then it succ
```
<team-user>@<jetson-host>:~$ echo $SSH_AUTH_SOCK
/tmp/ssh-XXXXLG1vwF/agent.6884
```
# Jetson testing
## test jetson can access github without setting local ssh key
```
ssh -T git@github.com
```
### if agent forwarding succ, then it will display:
```
Hi <team-github-account>! You've successfully authenticated, but GitHub does not provide shell access.
```

## pull test
```
<team-user>@<jetson-host>:~/wro_ign_gazebo_sim$ git pull origin main
remote: Enumerating objects: 45, done.
remote: Counting objects: 100% (45/45), done.
remote: Compressing objects: 100% (19/19), done.
remote: Total 35 (delta 19), reused 28 (delta 12), pack-reused 0 (from 0)
Unpacking objects: 100% (35/35), 6.28 KiB | 279.00 KiB/s, done.
From github.com:<team-github-account>/wro_ign_gazebo_sim
 * branch            main       -> FETCH_HEAD
   ae67267..d346e50  main       -> origin/main
Updating ae67267..d346e50
Fast-forward
 hiway_bringup_backup/bring-up-all-step.md                          |  67 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 hiway_bringup_backup/how-to-run-bringup.md                         |  23 ++++++++++++++++++++
 hiway_bringup_backup/launch/sensors.launch.py                      | 154 ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 hiway_bringup_backup/scripts/setup_ros_env.sh                      |  27 +++++++++++++++++++++++
 hiway_description/.DS_Store                                        | Bin 6148 -> 6148 bytes
 hiway_description/how-to-config-robot-localization.md              | 103 ---------------------------------------------------------------------------------------
 hiway_description/how-to-run-jetson.md                             |  74 ++-------------------------------------------------------------
 hiway_description/how-to-run-rtabmap-slam.md                       |  58 +++++++++++++++++++++++++++++++++++++++++++++++++
 hiway_description/how-to-run-windows-wsl.md                        |  10 ++++-----
 hiway_description/urdf/hiway.urdf.xacro                            |  32 ++++++++++++++++++++-------
 localization/config/ekf.yaml                                       |  16 ++++++++------
 localization/how-to-config-robot-localization.md                   | 157 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 {hiway_description => localization}/robot-localization-diagram.png | Bin
 13 files changed, 527 insertions(+), 194 deletions(-)
 create mode 100644 hiway_bringup_backup/bring-up-all-step.md
 create mode 100644 hiway_bringup_backup/how-to-run-bringup.md
 create mode 100644 hiway_bringup_backup/launch/sensors.launch.py
 create mode 100644 hiway_bringup_backup/scripts/setup_ros_env.sh
 delete mode 100644 hiway_description/how-to-config-robot-localization.md
 create mode 100644 hiway_description/how-to-run-rtabmap-slam.md
 create mode 100644 localization/how-to-config-robot-localization.md
 rename {hiway_description => localization}/robot-localization-diagram.png (100%)
<team-user>@<jetson-host>:~/wro_ign_gazebo_sim$
```

# All Done!
