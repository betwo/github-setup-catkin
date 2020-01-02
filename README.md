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

See [action.yml](action.yml)

Example using with two different compilers:
```yaml
runs-on: [ubuntu-18.04]
strategy:
  matrix:
    compiler: ["/usr/bin/g++", "/usr/bin/clang++"]
steps:
- uses: actions/checkout@v1
- uses: betwo/github-setup-catkin@v1.0.0
  with:
    ros-version: 'melodic'
    workspace: '$GITHUB_WORKSPACE'
- run: catkin_make_isolated -DCMAKE_C_COMPILER=${{ matrix.compiler }} -DCMAKE_CXX_COMPILER=${{ matrix.compiler }}
```

# License

The scripts and documentation in this project are released under the [MIT License](LICENSE)

# Contributions

Contributions are welcome!  See [Contributor's Guide](docs/contributors.md)
