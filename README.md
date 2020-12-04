# github-setup-catkin

<p align="left">
  <a href="https://github.com/betwo/github-setup-catkin"><img alt="GitHub Actions status" src="https://github.com/betwo/github-setup-catkin/workflows/Main%20workflow/badge.svg"></a>
</p>

This action sets up a ROS 1 environment for use of `catkin`.
This includes 
 * installing the requested ROS version
 * calling `rosdep install` on your repository
 * sourcing `setup.sh` for the following steps in the pipeline

# Usage

See [action.yml](action.yml) for available parameters.

## Example with a repository containing single package or  (top-level `package.xml`) or a full catkin workspace (`src`, `devel`, `build` are top-level)

```yaml
runs-on: [ubuntu-18.04]
steps:
- uses: actions/checkout@v1
- uses: betwo/github-setup-catkin@v1.1.1
  with:
    ros-version: 'melodic'
    workspace: '$GITHUB_WORKSPACE'
- run: catkin_make_isolated
```

## Example of using catkin_tools with a repository containing single package or  (top-level `package.xml`) or a full catkin workspace (`src`, `devel`, `build` are top-level)

```yaml
runs-on: [ubuntu-18.04]
steps:
- uses: actions/checkout@v1
- uses: betwo/github-setup-catkin@master
  with:
    ros-version: 'melodic'
    build-tool: 'catkin_tools'
    workspace: '$GITHUB_WORKSPACE'
- run: catkin build
```

## Example using a custom catkin workspace setup process (see also e.g. [here](https://github.com/betwo/csapex_core_plugins/blob/master/.github/workflows/catkin_make_isolated.yml):

```yaml
runs-on: [ubuntu-18.04]
steps:
- uses: actions/checkout@v1
  with:
    path: wworkspace/src/my_package
- uses: betwo/github-setup-catkin@v1.1.1
  with:
    ros-version: 'melodic'
    workspace: '$GITHUB_WORKSPACE'
- name: clone_dependencies
  run: |
    cd ..
    # clone more dependencies into the workspace, e.g.
    git clone https://github.com/betwo/csapex
    # ...
- uses: betwo/github-setup-catkin@v1.1.1
  with:
    ros-version: 'kinetic'
    workspace: $HOME/work/my-repo-name/workspace
- name: build
  run: |
    cd $HOME/work/my-repo-name/workspace
    catkin_make_isolated
```

## Example using with two different compilers:

```yaml
runs-on: [ubuntu-18.04]
strategy:
  matrix:
    compiler: ["/usr/bin/g++", "/usr/bin/clang++"]
steps:
- uses: actions/checkout@v1
- uses: betwo/github-setup-catkin@v1.1.1
  with:
    ros-version: 'melodic'
    workspace: '$GITHUB_WORKSPACE'
- run: catkin_make_isolated -DCMAKE_C_COMPILER=${{ matrix.compiler }} -DCMAKE_CXX_COMPILER=${{ matrix.compiler }}
```

# License

The scripts and documentation in this project are released under the [MIT License](LICENSE)

# Contributions

Contributions are welcome!  See [Contributor's Guide](docs/contributors.md)
