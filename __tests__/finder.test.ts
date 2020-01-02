import io = require('@actions/io');
import fs = require('fs');
import path = require('path');

const toolDir = path.join(
  __dirname,
  'runner',
  path.join(
    Math.random()
      .toString(36)
      .substring(7)
  ),
  'tools'
);
const tempDir = path.join(
  __dirname,
  'runner',
  path.join(
    Math.random()
      .toString(36)
      .substring(7)
  ),
  'temp'
);

process.env['RUNNER_TOOL_CACHE'] = toolDir;
process.env['RUNNER_TEMP'] = tempDir;

import * as finder from '../src/find-catkin';

describe('Finder tests', () => {
  it('Errors if catkin is not installed', async () => {
    let thrown = false;
    try {
      await finder.findCatkinVersion('melodic', toolDir);
    } catch {
      thrown = true;
    }
    expect(thrown).toBeTruthy();
  });

  it('Finds catkin_make_isolated if it is installed', async () => {
    const ros_bin: string = path.join(toolDir, 'ros', 'melodic', 'bin');
    await io.mkdirP(ros_bin);
    fs.writeFileSync(`${ros_bin}/catkin_make_isolated`, 'non-empty');
    let version = await finder.findCatkinVersion('melodic', toolDir);
    expect(version).toEqual('melodic');
  });

  it('Errors if the ROS version does not match', async () => {
    const ros_bin: string = path.join(toolDir, 'ros', 'melodic', 'bin');
    await io.mkdirP(ros_bin);
    fs.writeFileSync(`${ros_bin}/catkin_make_isolated`, 'non-empty');
    let thrown = false;
    try {
      await finder.findCatkinVersion('kinetic', toolDir);
    } catch {
      thrown = true;
    }
    expect(thrown).toBeTruthy();
  });
});
