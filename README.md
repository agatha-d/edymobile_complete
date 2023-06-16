
<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]

<br />
<div align="center">
  <a href="https://github.com/agatha-d/edymobile_complete">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">edymobile_complete</h3>

  <p align="center">
    This is a complete package to control the edymobile
    <br />
    <a href="https://github.com/agatha-d/edymobile_complete"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/agatha-d/edymobile_complete">View Demo</a>
    ·
    <a href="https://github.com/agatha-d/edymobile_complete/issues">Report Bug</a>
    ·
    <a href="https://github.com/agatha-d/edymobile_complete/issues">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#roadmap">Roadmap</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#ref">Useful resources</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

[![EdyMobile][robots-in-world]](https://github.com/agatha-d/edymobile_complete)

This project is a complete package to control a fleet of mobile differential drive robots, called Edymobiles, and allow them to transport samples from station to station in a chemistry laboratory for automating research experiments. The implementation is made in simulation using ROS and Gazebo. It has been developped as a part of a semester project for the SwissCAT+ laboratory and CREATE Lab at EPFL with the aim to automate chemistry research experiments.



### Built With

* [![ROS][Ros.org]][Ros-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

To start, learn about the urdf_tutorial

See the tutorials over at http://wiki.ros.org/urdf_tutorial

The robot description that includes the DiffDrive plugin to control the robot with velocity commands can be found at edymobile_complete/urdf/edymobile_theo.xacro. The configuration parameters for the joints of the model are to be fond in edymobile_complete/config/joints.yaml.

The Gazebo world used in the simulation is edymobile_complete/world/circuit_MAPF_VF.world.

The controller node is in edymobile_complete/src/edy_controller_node.py.

The fleet manager is in edymobile_complete/src/agatha_generate_goals.py.




### Prerequisites

* python
  ```sh
  sudo apt install python3-pip
  ```

### Installation

1. Clone the repo
   ```sh
   git clone https://github.com/agatha-d/edymobile_complete.git
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

To run the project:

* Launch the simulation in Gazebo that spawns the five Edymobile models in the world:
  ```sh
  roslaunch edymobile_complete edymobile_main.launch
  ```

You should see Gazebo open with the world representing the circuit in the laboratory and the five robots at their initial positions. So far, the robots do not move as the controllers have not been launched yet. This launch file opens Gazebo with the laboratory world and calls the spawn_edymobiles.launch file to spawn the different robots with the correct robot description and a namespace is associated with each robot (/robot1, /robot2, /robot3, /robot4 ad /robot5).

[![EdyMobile][robots-in-map]](https://github.com/agatha-d/edymobile_complete)


* Launch the controller nodes for all robots:
  ```sh
  roslaunch edymobile_complete controller_edymobiles.launch
  ```

This launch file launches an instance of the controller node for each of the five robots. The robots still do not move as the controller nodes are waiting for instructions from the fleet manager.

* Run the fleet manager script to send targets to the robots:
  ```sh
  cd edymobile_complete/src
  python3 agatha_generate_goals.py
  ```
The fleet manager will allocate tasks to different robots and generate a path for each robot to achieve those tasks. The next checkpoint along the path is sent to the controllers of each robots to which a task has been allocated, and these should start moving.



<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ROADMAP -->
## Roadmap

This project is the implementation of a basic framework for the control of a fleet of mobile robots. Future developments may include:

- [ ] Improving the control strategy to enable sharper turns at intersections accross the map
- [ ] Implement a waiting condition to avoid collision with a turning robot ahead
- [ ] Explore other task allocation strategies that could adapt robot velocities to deliver samples as fast as possible
- [ ] Establish metrics to evaluate the performance of the collective system, e.g. time needed to achieve all goals, number of active robots, etcEstablish metrics to evaluate the performance of the collective system, e.g. time needed to achieve all goals, number of active robots, etc.
- [ ] Adapt the code to work with an arbitrary number of robots
- [ ] Deploy on the real robots

See the [open issues](https://github.com/agatha-d/edymobile_complete/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USEFUL RESOURCES -->
## Useful Resources


* ROS Tutorials: http://wiki.ros.org/ROS/Tutorials
* Open Navigation LLC - Setting Up The URDF: https://navigation.ros.org/setup_guides/urdf/setup_urdf.html
* Gazebo ROS control: https://classic.gazebosim.org/tutorials?tut=ros_control&cat=connect_ros
* Robotic simulation scenarios with Gazebo and ROS: https://www.generationrobots.com/blog/en/robotic-simulation-scenarios-with-gazebo-and-ros/#create%20your%20own%20robot%20model
* Automatic Addison, Setting up the ROS navigation stack for a simulated robot: howpublished = "\url{https://automaticaddison.com/setting-up-the-ros-navigation-stack-for-a-simulated-robot/}"}





<!-- CONTACT -->
## Contact

Project Link: [https://github.com/agatha-d/edymobile_complete](https://github.com/agatha-d/edymobile_complete)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [EPFL](https://www.epfl.ch/en/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/github_username/repo_name.svg?style=for-the-badge
[contributors-url]: https://github.com/github_username/repo_name/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/github_username/repo_name.svg?style=for-the-badge
[forks-url]: https://github.com/agatha-d/edymobile_complete/forks
[stars-shield]: https://img.shields.io/github/stars/github_username/repo_name.svg?style=for-the-badge
[stars-url]:https://github.com/agatha-d/edymobile_complete/stargazers
[issues-shield]: https://img.shields.io/github/issues/github_username/repo_name.svg?style=for-the-badge
[issues-url]: https://github.com/agatha-d/edymobile_complete/issues
[license-shield]: https://img.shields.io/github/license/github_username/repo_name.svg?style=for-the-badge
[license-url]: https://github.com/github_username/repo_name/blob/master/LICENSE.txt
[product-screenshot]: images/screenshot.png
[robots-in-map]: images/map.png
[robots-in-world]: images/robots.png
[Ros.org]: https://img.shields.io/ros/v/humble/vision_msgs
[Ros-url]: https://www.ros.org/
