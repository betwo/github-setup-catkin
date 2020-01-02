import * as path from 'path';
import * as core from '@actions/core';
import fs = require('fs');

export async function findCatkinVersion(
  version: string,
  prefix: string = '/opt'
): Promise<string> {
  let _binDir = `${prefix}/ros/${version}/bin/`;

  if (!fs.existsSync(_binDir)) {
    // PyPy not installed in $(Agent.ToolsDirectory)
    throw new Error(`ROS version ${version} not found`);
  }

  let _rosPrefix = path.dirname(_binDir);
  core.exportVariable('ros_prefix', _rosPrefix);

  core.addPath(_binDir);

  return path.basename(_rosPrefix);
}
