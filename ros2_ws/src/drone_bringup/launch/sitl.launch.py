"""Bring up PX4 SITL (Gazebo Harmonic) and the Micro XRCE-DDS agent.

    ros2 launch drone_bringup sitl.launch.py                      # gz_x500, GUI
    ros2 launch drone_bringup sitl.launch.py vehicle:=gz_x500_depth
    ros2 launch drone_bringup sitl.launch.py headless:=true       # no Gazebo GUI (CI / tests)
    ros2 launch drone_bringup sitl.launch.py px4_params:=NAV_DLL_ACT=2   # keep GCS-loss failsafe (needs QGC)

PX4 is started the same way `make px4_sitl <vehicle>` does (binary run from
build/px4_sitl_default/rootfs with PX4_SIM_MODEL=<vehicle>), but in daemon mode (`-d`) so no
interactive pxh shell is spawned (without a TTY the shell would spin on EOF and flood the log).
Build PX4 first with scripts/setup/02_px4.sh.
Phase 2 will add px4_bridge / RViz nodes here; Phase 3 the sensor adapter.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# SITL-only parameter overrides (applied after boot through the px4-param daemon client):
#  NAV_DLL_ACT=0     gz_x500 airframe enables the datalink-loss failsafe, which makes
#                    "No connection to the GCS" a pre-arm failure. No QGC in automated tests.
#  COM_OF_LOSS_T=3   offboard-loss timeout [s] (default 1). PX4 converts incoming timestamps
#                    with the XRCE timesync offset; on WSL2 the wall clock is stepped backwards by
#                    up to ~0.8 s every ~10 s, which looked like a lost offboard stream (-> RTL).
DEFAULT_PX4_PARAMS = 'NAV_DLL_ACT=0,COM_OF_LOSS_T=3'


def _setup(context, *args, **kwargs):
    vehicle = LaunchConfiguration('vehicle').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() in ('1', 'true', 'yes')
    world = LaunchConfiguration('world').perform(context)
    agent_port = LaunchConfiguration('agent_port').perform(context)
    px4_dir = LaunchConfiguration('px4_dir').perform(context)
    px4_params = LaunchConfiguration('px4_params').perform(context)

    px4_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4')
    px4_param_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4-param')
    rootfs = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'rootfs')
    if not os.path.isfile(px4_bin):
        raise RuntimeError(f'PX4 SITL binary not found: {px4_bin} (run scripts/setup/02_px4.sh / make px4_sitl)')

    env = dict(os.environ)
    env['PX4_SIM_MODEL'] = vehicle          # e.g. gz_x500 -> airframe 4001_gz_x500
    env['GZ_IP'] = '127.0.0.1'
    if headless:
        env['HEADLESS'] = '1'
    if world:
        env['PX4_GZ_WORLD'] = world
    # Parameter overrides are applied AFTER boot through the px4-param daemon client.
    # (PX4_PARAM_* env vars are applied before the airframe file, whose `param set-default`
    #  silently overrides any value equal to the compiled default - e.g. NAV_DLL_ACT=0.)
    param_cmds = []
    for item in [x.strip() for x in px4_params.split(',') if x.strip()]:
        name, _, value = item.partition('=')
        if not value:
            raise RuntimeError(f'px4_params entry "{item}" must be NAME=VALUE')
        param_cmds.append(f'{px4_param_bin} set {name.strip()} {value.strip()}')

    px4 = ExecuteProcess(
        cmd=[px4_bin, '-d'],                # -d: daemon mode, no pxh shell
        cwd=rootfs,
        env=env,
        name='px4_sitl',
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='5',
    )
    agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', agent_port],
        name='xrce_agent',
        output='screen',
    )
    actions = [
        LogInfo(msg=f'[sitl.launch] PX4={px4_dir} vehicle={vehicle} headless={headless} agent_port={agent_port} px4_params={px4_params}'),
        agent,
        px4,
    ]
    if param_cmds:
        # Wait until the daemon answers (`px4-param show` succeeds), then apply every override.
        script = (f'for i in $(seq 1 60); do {px4_param_bin} show -q SYS_AUTOSTART >/dev/null 2>&1 && break; sleep 1; done; '
                  + ' && '.join(param_cmds)
                  + ' && echo "[sitl.launch] applied px4_params: ' + px4_params + '"')
        actions.append(ExecuteProcess(
            cmd=['bash', '-c', script],
            cwd=rootfs,                     # px4-param finds the daemon socket relative to cwd
            name='px4_params',
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='gz_x500',
                              description='PX4 SITL target: gz_x500 | gz_x500_depth | gz_x500_vision ...'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true: run Gazebo without GUI (HEADLESS=1)'),
        DeclareLaunchArgument('world', default_value='',
                              description='PX4_GZ_WORLD name (empty = PX4 default world)'),
        DeclareLaunchArgument('agent_port', default_value='8888',
                              description='Micro XRCE-DDS agent UDP port'),
        DeclareLaunchArgument('px4_params', default_value=DEFAULT_PX4_PARAMS,
                              description='Comma-separated PX4 parameter overrides NAME=VALUE applied at boot'),
        DeclareLaunchArgument('px4_dir', default_value=os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot')),
                              description='PX4-Autopilot checkout'),
        OpaqueFunction(function=_setup),
    ])
