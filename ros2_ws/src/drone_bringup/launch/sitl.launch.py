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
#  SIM_BAT_DRAIN=3600 simulated battery drains fully after SIM_BAT_DRAIN seconds of being armed (default 60 s!)
#                    and then "Battery unhealthy" blocks re-arming; SIM_BAT_MIN_PCT=90 keeps it above 90 %.
DEFAULT_PX4_PARAMS = 'NAV_DLL_ACT=0,COM_OF_LOSS_T=3,SIM_BAT_DRAIN=3600,SIM_BAT_MIN_PCT=90'

# vehicles defined in simulation/models/ -> PX4 airframe id to boot with
CUSTOM_VEHICLE_AUTOSTART = {'gz_x500_rgbd': '4002'}


def _repo_root():
    # install/drone_bringup/share/drone_bringup/launch -> repo root is 6 levels up from the install dir layout
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        if os.path.isdir(os.path.join(here, 'simulation', 'models')) and os.path.isdir(os.path.join(here, 'ros2_ws')):
            return here
        here = os.path.dirname(here)
    return os.environ.get('ISE_DRONE_ROOT', '')


def _link_assets(px4_dir):
    """Symlink simulation/models/* and simulation/worlds/*.sdf into PX4's gz tree (idempotent)."""
    root = _repo_root()
    if not root:
        return
    for sub, pattern in (('models', None), ('worlds', '.sdf')):
        src_dir = os.path.join(root, 'simulation', sub)
        dst_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', sub)
        if not (os.path.isdir(src_dir) and os.path.isdir(dst_dir)):
            continue
        for name in os.listdir(src_dir):
            if pattern and not name.endswith(pattern):
                continue
            src, dst = os.path.join(src_dir, name), os.path.join(dst_dir, name)
            if os.path.islink(dst) and os.readlink(dst) == src:
                continue
            if not os.path.exists(dst):
                os.symlink(src, dst)


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
    # Repo-provided models (simulation/models/*) have no PX4 airframe file; reuse the x500_depth airframe (4002)
    # and make sure they are linked into the PX4 model tree (scripts/setup/05_sim_assets.sh does the same).
    airframes = os.path.join(rootfs, 'etc', 'init.d-posix', 'airframes')
    if os.path.isdir(airframes) and not any(f.endswith('_' + vehicle) for f in os.listdir(airframes)):
        env['PX4_SYS_AUTOSTART'] = CUSTOM_VEHICLE_AUTOSTART.get(vehicle, '4002')
    _link_assets(px4_dir)
    if headless:
        env['HEADLESS'] = '1'
    if world:
        env['PX4_GZ_WORLD'] = world
    # Parameter overrides are applied AFTER boot through the px4-param daemon client.
    # (PX4_PARAM_* env vars are applied before the airframe file, whose `param set-default`
    #  silently overrides any value equal to the compiled default - e.g. NAV_DLL_ACT=0.)
    param_cmds, param_pairs = [], []
    for item in [x.strip() for x in px4_params.split(',') if x.strip()]:
        name, _, value = item.partition('=')
        if not value:
            raise RuntimeError(f'px4_params entry "{item}" must be NAME=VALUE')
        param_pairs.append((name.strip(), value.strip()))
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
        # The daemon socket answers while rcS is still running; the airframe file's `param set-default`
        # would then overwrite our values. Wait for the logger (started last in rcS), apply, verify, repeat.
        px4_logger_bin = os.path.join(px4_dir, 'build', 'px4_sitl_default', 'bin', 'px4-logger')
        verify = ' && '.join(f'{px4_param_bin} compare -s {name} {value}' for name, value in param_pairs)
        script = (f'for i in $(seq 1 90); do {px4_logger_bin} status >/dev/null 2>&1 && break; sleep 1; done; sleep 2; '
                  f'for i in $(seq 1 10); do {" && ".join(param_cmds)} >/dev/null; sleep 1; '
                  f'if {verify}; then echo "[sitl.launch] applied px4_params: {px4_params}"; exit 0; fi; done; '
                  f'echo "[sitl.launch] ERROR: px4_params not applied"; exit 1')
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
                              description='PX4 SITL target: gz_x500 | gz_x500_depth | gz_x500_rgbd (repo model) ...'),
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
