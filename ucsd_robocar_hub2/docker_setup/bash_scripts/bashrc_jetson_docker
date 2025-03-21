# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
[ -z "$PS1" ] && return

# don't put duplicate lines in the history. See bash(1) for more options
# ... or force ignoredups and ignorespace
HISTCONTROL=ignoredups:ignorespace

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "$debian_chroot" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

function git_branch {
  git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'
}

function markup_git_branch {
  if [[ "x$1" = "x" ]]; then
    echo -e "$1"
  else
    if [[ $(git status 2> /dev/null | tail -n1) = "nothing to commit, working tree clean" ]]; then
      echo -e '\033[1;32m('"$1"')\033[0;0m'
    else
      echo -e '\033[1;31m('"$1"')\033[0;0m'
    fi
  fi
}

COLOR_RED="\033[0;31m"
COLOR_YELLOW="\033[0;33m"
COLOR_GREEN="\033[0;32m"
COLOR_OCHRE="\033[38;5;95m"
COLOR_BLUE="\033[0;34m"
COLOR_WHITE="\033[0;37m"
COLOR_RESET="\033[0m"


if [ "$color_prompt" = yes ]; then
    PS1='\[\033[01;34m\]Docker_Container\[\033[01;31m\]@\[\033[01;34m\]\h\[\033[01;34m\]:\[\033[01;32m\]\w\[\033[00m\]$(markup_git_branch $(git_branch))🐳 '
    # PS1='\[\033[01;34m\]Docker_Container@ucsd_robocar\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]$(markup_git_branch $(git_branch))🐳 '
    #PS1='\[\033[01;31m\]Docker_Container@ucsd_robocar\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\] 🐳 '
    #PS1='🐳${debian_chroot:+($debian_chroot)}\[\033[01;31m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
# \[\033[01;31m\]@\[\033[01;34m\]\h\[\033[01;34m\]:
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.alias
# See /usr/share/doc/bash-doc/examples in the bash-doc package.
#/home/projects/f110_ws/devel
if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
#if [ -f /etc/bash_completion ] && ! shopt -oq posix; then
#    . /etc/bash_completion
#fi
cd /home/projects

function source_ros1_pkg() {
  source /opt/ros/noetic/setup.bash
  source /home/projects/sensor1_ws/src/lidars/ld06/devel/setup.bash
  cd /home/projects/ros1_ws
  source devel/setup.bash
}

function source_ros1_init() {
  source_ros1_pkg
  rm -rf build/ devel/
  catkin_make
  source devel/setup.bash
  roscore
}

function source_ros1() {
  source_ros1_pkg
  cd /home/projects/ros1_ws
}

function source_ros2_pkg() {
  source /opt/ros/foxy/setup.bash
  source /home/projects/sensor2_ws/src/cameras/oakd/install/setup.bash
  source /home/projects/sensor2_ws/src/imu/artemis_openlog/install/setup.bash
  source /home/projects/sensor2_ws/src/vesc/install/setup.bash
  source /home/projects/sensor2_ws/src/lidars/ld06/ros2/install/setup.bash
  source /home/projects/rosboard_ws/install/setup.bash
}

function source_ros2() {
  source_ros2_pkg
  cd /home/projects/ros2_ws
}

function build_ros2() {
  cd /home/projects/ros2_ws 
  rm -rf build/ install/ log/ 
  colcon build 
  source install/setup.bash
}

function source_ros_bridge() {
  source /opt/ros/noetic/setup.bash
  source /opt/ros/foxy/setup.bash
  source /home/projects/ros2_ws/install/setup.bash
  ros2 launch ucsd_robocar_nav2_pkg ros_bridge_launch.launch.py
}

function upd_ucsd_robocar() {
  cd /home/projects/ros2_ws/src/ucsd_robocar_hub2
  git submodule update --remote --merge
  build_ros2
}

export ROS_DOMAIN_ID=96
