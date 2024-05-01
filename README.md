# iot-repo-1: Single-person Household Smart Home Care Service
> IoT project team 1: ALT F4 <br>
## Project Introduction
- **Topic**: Single-person Household Care Service
- **Objective**: Implement a system for analyzing the risk level within single-person households to prevent and respond to safety accidents.
## Technologies
| Category       | Details                                                               |
|----------------|-----------------------------------------------------------------------|
| Development Environment | <img src="https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white"> <img src="https://img.shields.io/badge/Arduino-00878F?style=for-the-badge&logo=arduino">|
| Collaboration Tools      | <img src="https://img.shields.io/badge/GiHub-181717?style=for-the-badge&logo=github">  <img src="https://img.shields.io/badge/slack-4A154B?style=for-the-badge&logo=slack&logoColor=white"> <img src="https://img.shields.io/badge/Draw.io-F08705?style=for-the-badge">  <img src="https://img.shields.io/badge/Jira-0052CC?style=for-the-badge&logo=jira">|
| GUI                      | <img src="https://img.shields.io/badge/PyQt5-21C25E?style=for-the-badge&logo=quicktype">|
| Web Crawling             | <img src="https://img.shields.io/badge/selenium-43B02A?style=for-the-badge&logo=selenium&logoColor=white">|
| Data Management          | <img src="https://img.shields.io/badge/OpenCV-5C3EE8?style=for-the-badge&logo=OpenCV">  <img src="https://img.shields.io/badge/pandas-150458?style=for-the-badge&logo=pandas&logoColor=white">  <img src="https://img.shields.io/badge/mysql-4479A1?style=for-the-badge&logo=mysql&logoColor=white">|
## Team Introduction
| Role        | Name         | Duties                                      |
|-------------|--------------|---------------------------------------------|
| Team Leader | Jaehoon Shin |IOManager implementation and HW configuration, data collection and DB management, collaboration tool setting |
| Team Member | Yohan Kim    | CareService GUI implementation, DB construction and storage, risk measurement and automatic mode implementation |
| Team Member | Youngsoo Son | CareService GUI implementation and TCP communication construction, 3D house modeling construction |
| Team Member | Hyunbok Lee  | HomeService GUI implementation and TCP communication establishment, DB storage |

## System Architecture
![image](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/a405b802-879b-45e0-99e4-ab16f518d044)
## Sequence Diagram
- Danger step

| Step | Detection       | Response            |
|------|-----------------|---------------------|
| LV 0 | Gas and dust    | Automatic mode operation, GUI representation |
| LV 1 | Unusual sounds  | GUI representation |
| LV 2 | Loud sound without motion | Raise warning message |
| LV 3 | No response to warning message | Provide camera image and report |

![image](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/b1d9aadc-4354-437b-b919-09d8c50ff8f0)

![image](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/26e87c6b-b6bd-4272-b2fe-aac195c8895e)

![image](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/ebdbabd4-1545-496e-a49f-5d642f7ae0cd)

![image](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/688ba3a2-8dce-4e82-8fc7-a1816826fc95)



## GUI
![image](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/62442255-b14d-449f-ae7e-e9717c879e68)


![screenshot_from_2024-05-01_09-19-28](https://github.com/addinedu-ros-5th/iot-repo-1/assets/163791820/615db182-b2cb-4fd5-8dcc-5b912f11ec86)

## Hardware Configuration
- 3D modeling
![Home sensor](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/4bccb56c-d664-4a73-b927-31cc08110b95)
- Floor plan
![20240425_130101](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/0b31c4b8-1b1a-4730-86f9-0710d2f08686)

## Database
![image](https://github.com/addinedu-ros-5th/iot-repo-1/assets/86091697/9e66fc68-5210-473c-94e1-2ceec4b37a72)

## Improvements

1. Security Issues:
- Privacy Compliance: Ensure that camera installations are designed in compliance with privacy regulations and ethical considerations.
- Master Key Management: Implement a robust system for managing master keys, including encryption, key rotation, and access control policies.
2. Choosing an Appropriate Topic:
- Cost-Benefit Analysis: Conduct a thorough cost-benefit analysis to assess the feasibility and potential impact of the selected topic.
3. Analysis of Various Scenarios:
- Consider a variety of scenarios, including multiple risk scenarios, during the analysis process.
