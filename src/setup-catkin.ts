import * as core from '@actions/core';
import * as finder from './find-catkin';
import child_process = require('child_process');

function getBuildToolPackages(build_tool: string): string[] {
  switch (build_tool) {
    case 'catkin':
      return ['python-catkin-pkg', 'python3-catkin-pkg'];
    case 'catkin_tools':
      return [
        'python-catkin-tools',
        'python3-catkin-tools python3-osrf-pycommon'
      ];
    default:
      throw new Error(`build tool ${build_tool} cannot be installed`);
  }
}

function installRos(version: string, build_tool: string) {
  const [build_tool_py2, build_tool_py3] = getBuildToolPackages(build_tool);

  const command = `export ACTIONS_ALLOW_UNSECURE_COMMANDS=true && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && sudo apt-get update && sudo apt-get -qq update -y && ( sudo apt-get -qq install build-essential openssh-client ros-${version}-ros-base ${build_tool_py3} python3-rosdep -y ||sudo apt-get -qq install build-essential openssh-client ros-${version}-ros-base ${build_tool_py2} python-rosdep -y ; ) && sudo rosdep init && rosdep update --include-eol-distros`;
  child_process.execSync(command, {stdio: 'inherit'});
}
function rosdepInstall(workspace_root: string, version: string) {
  let command = `. /opt/ros/${version}/setup.sh &&
cd ${workspace_root} &&
rosdep install --from-paths --reinstall --ignore-packages-from-source --default-yes --verbose .`;
  child_process.execSync(command, {stdio: 'inherit'});
}

function makeInitCommand(build_tool: string) {
  switch (build_tool) {
    case 'catkin':
      return `catkin_init_workspace`;
    case 'catkin_tools':
      return `catkin init`;
    default:
      throw new Error(`build tool ${build_tool} cannot be initialized`);
  }
}

function sourceWorkspace(
  workspace_root: string,
  version: string,
  build_tool: string
) {
  const init_workspace_command = makeInitCommand(build_tool);
  const command = `. /opt/ros/${version}/setup.sh &&
cd ${workspace_root} &&
if [ -f package.xml ]; then
  mkdir tmp-src;
  mv * tmp-src;
  mv tmp-src src;
fi
  ${init_workspace_command};
env`;
  let options: child_process.ExecSyncOptionsWithStringEncoding = {
    encoding: 'utf8'
  };
  let env: string = child_process.execSync(command, options);

  console.log(`Environment: ${env}`);
  let assignments = env.split(/\n/);
  for (let assignment of assignments) {
    let [name, value] = assignment.split('=');
    if (
      name !== undefined &&
      name != '' &&
      value !== undefined &&
      value != ''
    ) {
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

    let build_tool = core.getInput('build-tool');
    if (!build_tool) {
      build_tool = 'catkin';
    }

    let installed_version: undefined | string;
    try {
      console.log(`Checking if ROS ${requested_version} is installed`);
      installed_version = await finder.findCatkinVersion(requested_version);
    } catch (error) {
      console.log(`Installing ROS ${requested_version}`);
      installRos(requested_version, build_tool);
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

    console.log(`Sourcing workspace with build tool ${build_tool}`);
    sourceWorkspace(workspace_root, installed_version, build_tool);
  } catch (err) {
    core.setFailed(err.message);
  }
}

run();
