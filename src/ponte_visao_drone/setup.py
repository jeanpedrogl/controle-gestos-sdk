from setuptools import find_packages, setup

package_name = "ponte_visao_drone"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sistema.launch.py"]),
    ],
    package_data={package_name: ["modelos/*.task"]},
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jean Pedro Grossi Líbero",
    maintainer_email="jeanpedrogrossi@gmail.com",
    description=(
        "Visão computacional por gestos rodando no computador; ponte ROS2 "
        "para o celular controlar o drone DJI Mini SE."
    ),
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "piloto_gestos = ponte_visao_drone.integracao_visao_drone:main",
            "visor_camera_drone = ponte_visao_drone.visor_camera_drone:main",
        ],
    },
)
