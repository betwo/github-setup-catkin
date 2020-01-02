import * as core from '@actions/core';
import * as finder from './find-catkin';
import child_process = require('child_process');

async function installRos(version: string) {
  let command = `sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 &&
sudo apt-get update &&
sudo apt-get -qq update -y && sudo apt-get -qq install build-essential openssh-client ros-${version}-ros-base python-catkin-pkg -y &&
sudo rosdep init &&
rosdep update
`;
  child_process.execSync(command, {stdio: 'inherit'});
}
async function rosdepInstall(workspace_root: string, version: string) {
  let command = `. /opt/ros/${version}/setup.sh &&
cd ${workspace_root} &&
rosdep install --from-paths -i -y src`;
  child_process.execSync(command, {stdio: 'inherit'});
}

async function sourceWorkspace(workspace_root: string, version: string) {
  let command = `. /opt/ros/${version}/setup.sh &&
cd ${workspace_root} &&
catkin_init_workspace &&
env`;
  let options: child_process.ExecSyncOptionsWithStringEncoding = {
    encoding: 'utf8'
  };
  let env: string = child_process.execSync(command, options);

  console.log(`Environment: ${env}`);
  let assignments = env.split(/\n/);
  for (let assignment of assignments) {
    let [name, value] = assignment.split('=');
    if (name !== undefined && value !== undefined) {
      console.log(`${name} = ${value}`);
      core.exportVariable(name, value);
    }
  }
}

async function run() {
  try {
    let workspace_root = core.getInput('workspace');
    if (!workspace_root) {
      throw Error(`The workspace root must be specified via 'workspace'.`);
    }

    let requested_version = core.getInput('ros-version');
    if (!requested_version) {
      requested_version = 'melodic';
    }

    let installed_version: undefined | string;
    try {
      console.log(`Checking if ROS ${requested_version} is installed`);
      installed_version = await finder.findCatkinVersion(requested_version);
    } catch (error) {
      console.log(`Installing ROS ${requested_version}`);
      installRos(requested_version);
      installed_version = await finder.findCatkinVersion(requested_version);
    }

    console.log(`ROS ${installed_version} is installed`);
    if (installed_version !== requested_version) {
      throw Error(
        `Installed ROS version '${installed_version}' is not '${requested_version}'`
      );
    }

    console.log(`Installing rosdep dependencies`);
    rosdepInstall(workspace_root, installed_version);

    console.log(`Sourcing workspace`);
    sourceWorkspace(workspace_root, installed_version);
  } catch (err) {
    core.setFailed(err.message);
  }
}

run();
