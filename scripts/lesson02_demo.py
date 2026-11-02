#!/usr/bin/env python3
"""在正常桌面执行的真实联调/录屏；不会把模拟图像或日志冒充图形验收。"""
import argparse
import csv
import datetime as dt
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
from lesson02_recorder import ScreenCapture

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=int, default=45, help='手动操纵和画面演示时长，默认45秒')
    parser.add_argument('--prepare-seconds', type=int, default=30, help='摆放窗口的准备时间，默认30秒，不计入操纵时长')
    parser.add_argument('--no-record', action='store_true', help='仅诊断；不录屏时整体为 PARTIAL')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 300:
        parser.error('--seconds 必须在 10 到 300 之间')
    if not 0 <= args.prepare_seconds <= 180:
        parser.error('--prepare-seconds 必须在 0 到 180 之间')
    output = ROOT / 'build/lesson02_evidence' / dt.datetime.now().strftime('desktop_%Y%m%d_%H%M%S')
    output.mkdir(parents=True)
    report = {'status': 'PARTIAL', 'directory': str(output), 'checks': {}, 'errors': []}
    children, threads, handles = [], [], []
    node = None
    ros = None
    recorder = None
    capture = ScreenCapture()
    camera_health = {'samples': 0, 'min_height': None, 'below_scene': False}
    clean_env = os.environ.copy()
    for key in ('LD_LIBRARY_PATH', 'PYTHONPATH', 'GST_PLUGIN_PATH', 'GST_PLUGIN_SYSTEM_PATH'):
        clean_env.pop(key, None)
    clean_env['LC_ALL'] = 'C'

    def log(message):
        print(message, flush=True)
        with (output / 'session.log').open('a') as stream:
            stream.write(message + '\n')

    def start(name, command, env=None, echo=True, pass_fds=()):
        stream = (output / (name + '.log')).open('w')
        handles.append(stream)
        proc = subprocess.Popen(command, cwd=ROOT, env=env, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, text=True, bufsize=1,
                                start_new_session=True, pass_fds=pass_fds)
        children.append(proc)

        def consume():
            for line in proc.stdout:
                stream.write(line)
                stream.flush()
                if name == 'simulator' and 'CAMERA_STATE' in line:
                    match = re.search(r'position=Vec3\([^,]+,\s*([^,]+),', line)
                    if match:
                        height = float(match.group(1))
                        camera_health['samples'] += 1
                        if math.isfinite(height):
                            previous = camera_health['min_height']
                            camera_health['min_height'] = height if previous is None else min(previous, height)
                        if not math.isfinite(height) or height < -2:
                            camera_health['below_scene'] = True
                # 录屏中保留培训程序的原始 Open/每秒反馈/Close，过滤高频 Read 计数。
                visible_training = name in ('training', 'no-data-training') and (
                    line.startswith(('[IO Open]', '[IO Close]', 'yaw=', 'training:')))
                if echo or visible_training:
                    print('[' + name + '] ' + line, end='', flush=True)
        thread = threading.Thread(target=consume, daemon=True)
        thread.start()
        threads.append(thread)
        return proc

    def stop(proc, timeout=8):
        if proc is None:
            return None
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        return proc.returncode

    def check_camera_height():
        if camera_health['below_scene']:
            report['checks']['camera_stayed_above_ground'] = False
            if data['last_image'] is not None:
                save_image(data['last_image'], 'camera-last.png')
            raise RuntimeError('机器人／相机已掉到训练场地板下方，停止本轮录屏验收；请查看 simulator.log 的 CAMERA_STATE，勿继续提交灰屏视频。')

    try:
        log('宝贝，这个命令会自动录屏，无须另外启动录屏软件。')
        log('录制流程：选择整个屏幕 → 自动启动 → 点击有场地的仿真器 → 按方向键 → 自动结束并保存。')
        log('结果目录：' + str(output))
        display = subprocess.run(['/usr/bin/xdpyinfo'], env=clean_env, text=True,
                                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
        (output / 'display.log').write_text(display.stdout)
        if display.returncode:
            raise RuntimeError('无法连接桌面显示：' + display.stdout.strip())

        if not args.no_record:
            source_plugin = 'pipewiresrc' if os.environ.get('WAYLAND_DISPLAY') else 'ximagesrc'
            for plugin in (source_plugin, 'vp8enc', 'webmmux'):
                subprocess.run(['/usr/bin/gst-inspect-1.0', plugin], env=clean_env,
                               stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, check=True, timeout=5)
            source, descriptors = capture.source()
            recorder = start('recorder', ['/usr/bin/gst-launch-1.0', '-e', *source, '!',
                'videoconvert', '!', 'videorate', '!', 'video/x-raw,framerate=10/1', '!', 'videoscale', '!',
                'video/x-raw,width=1600,height=900,pixel-aspect-ratio=1/1', '!', 'videoconvert', '!',
                'vp8enc', 'deadline=1', 'cpu-used=8', 'threads=2', 'target-bitrate=3000000', '!',
                'webmmux', '!', 'filesink', 'location=' + str(output / 'lesson02-demo.webm')],
                clean_env, False, descriptors)
            time.sleep(0.5)
            if recorder.poll() is not None:
                raise RuntimeError('录屏程序启动失败，请查看 recorder.log')
            log('● 正在自动录屏，结束后保存到：' + str(output / 'lesson02-demo.webm'))
        else:
            log('当前是 --no-record 诊断模式，没有录屏。')

        log('阶段 1：验证数据契约、ROS2 节点收发、禁控与禁火配置。')
        tests = subprocess.run(['ctest', '--test-dir', 'build', '--output-on-failure'], cwd=ROOT, text=True,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=35)
        (output / 'tests.log').write_text(tests.stdout)
        report['checks']['tests'] = tests.returncode == 0
        if tests.returncode:
            raise RuntimeError('测试未通过，见 tests.log')
        log('基础测试通过；详细日志已保存，不再滚屏显示。')

        import rclpy
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from hnurm_interfaces.msg import VisionRecvData, VisionSendData
        from PIL import Image as PillowImage, ImageStat
        ros = rclpy
        ros.init(args=[])
        node = ros.create_node('lesson02_acceptance_probe')
        for _ in range(6):
            ros.spin_once(node, timeout_sec=0.05)
        if 'training_io' in node.get_node_names():
            raise RuntimeError('检测到已经运行的 /training_io；请先关闭旧培训程序再运行演示')
        if node.count_publishers('/image_raw') or node.count_publishers('/vision_receive_data'):
            raise RuntimeError('检测到旧图像或反馈发布者；请先关闭旧仿真器再运行演示')
        data = {'images': 0, 'feedback': 0, 'commands': 0, 'states': set(),
                'yaw_travel': 0., 'pitch_min': None, 'pitch_max': None,
                'last_yaw': None, 'last_output_ns': 0, 'last_state': None, 'last_image': None,
                'scene_detail': False, 'image_detail_stddev': 0., 'last_detail_check': 0.,
                'speed': 0., 'pitch': 0., 'last_feedback_status': 0.}
        feedback_csv = (output / 'feedback.csv').open('w', newline='')
        command_csv = (output / 'commands.csv').open('w', newline='')
        handles.extend([feedback_csv, command_csv])
        fw, cw = csv.writer(feedback_csv), csv.writer(command_csv)
        fw.writerow(['stamp_ns', 'yaw_deg', 'pitch_deg', 'roll_deg', 'bullet_speed_mps'])
        cw.writerow(['stamp_ns', 'state', 'yaw_deg', 'pitch_deg', 'distance'])

        def pillow_image(msg):
            if msg.encoding not in ('rgb8', 'bgr8'):
                raise RuntimeError('未预期的仿真图像格式：' + msg.encoding)
            return PillowImage.frombytes('RGB', (msg.width, msg.height), bytes(msg.data),
                                        'raw', 'RGB' if msg.encoding == 'rgb8' else 'BGR', msg.step, 1)

        def save_image(msg, name):
            pillow_image(msg).save(output / name)

        def on_image(msg):
            data['images'] += 1
            data['last_image'] = msg
            if data['images'] == 1:
                save_image(msg, 'camera-first.png')
            if time.monotonic() - data['last_detail_check'] >= 1:
                # 缩小后统计空间细节，降低传感器噪声影响；这只是一项空白画面预检。
                small = pillow_image(msg).resize((96, 72), getattr(PillowImage, 'Resampling', PillowImage).BOX)
                data['image_detail_stddev'] = max(ImageStat.Stat(small).stddev)
                data['scene_detail'] = data['image_detail_stddev'] >= 3
                data['last_detail_check'] = time.monotonic()

        def on_feedback(msg):
            data['feedback'] += 1
            if data['last_yaw'] is not None:
                data['yaw_travel'] += abs(math.remainder(msg.yaw - data['last_yaw'], 360))
            data['last_yaw'] = msg.yaw
            data['pitch'] = msg.pitch
            data['speed'] = msg.bullet_speed.data
            data['pitch_min'] = msg.pitch if data['pitch_min'] is None else min(data['pitch_min'], msg.pitch)
            data['pitch_max'] = msg.pitch if data['pitch_max'] is None else max(data['pitch_max'], msg.pitch)
            fw.writerow([msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec,
                         msg.yaw, msg.pitch, msg.roll, msg.bullet_speed.data])

        def on_command(msg):
            data['commands'] += 1
            data['states'].add(msg.target_state.data)
            data['last_output_ns'] = msg.header.stamp.sec*10**9 + msg.header.stamp.nanosec
            data['last_state'] = msg.target_state.data
            cw.writerow([data['last_output_ns'], msg.target_state.data, msg.yaw, msg.pitch, msg.target_distance])

        node.create_subscription(Image, '/image_raw', on_image, qos_profile_sensor_data)
        node.create_subscription(VisionRecvData, '/vision_receive_data', on_feedback, qos_profile_sensor_data)
        node.create_subscription(VisionSendData, '/vision_send_data', on_command, 20)

        def spin(seconds):
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                ros.spin_once(node, timeout_sec=0.05)

        # 培训程序本身不发布图像/反馈，先独立验证无数据时可 Ctrl+C 正常退出。
        cfg = (ROOT / 'configs/simulation.yaml').read_text()
        if not re.search(r'^allow_fire:\s*false\s*$', cfg, re.M):
            raise RuntimeError('演示要求默认配置 allow_fire: false')
        if not re.search(r'^display:\s*true\s*$', cfg, re.M):
            raise RuntimeError('录屏演示要求配置 display: true')
        headless = output / 'headless.yaml'
        headless.write_text(re.sub(r'^display:.*$', 'display: false', cfg, flags=re.M))
        idle = start('no-data-training', [str(ROOT / 'build/training'), str(headless)], echo=False)
        spin(1.5)
        idle_code = stop(idle)
        spin(0.3)
        idle_log = (output / 'no-data-training.log').read_text()
        report['checks']['no_data_ctrl_c'] = (idle_code == 0 and '[IO Close]' in idle_log and
                not any(int(n) > 0 for n in re.findall(r'frames=(\d+)', idle_log)))
        log('无数据 Ctrl+C 检查：' + ('PASS' if report['checks']['no_data_ctrl_c'] else 'FAIL') +
            '；exit_code=' + str(idle_code) + '；原始日志：no-data-training.log')

        log('阶段 2：启动真实仿真器、相机显示和停止指令发布。')
        run_dir = output / 'simulator-runtime'
        run_dir.mkdir()
        sim_config = (ROOT / 'simulation/simulator/config.toml').read_text()
        sim_config, changes = re.subn(r'(?m)^auto_aim_enabled\s*=\s*(?:true|false)\s*$',
                                      'auto_aim_enabled = false', sim_config)
        if changes != 1:
            raise RuntimeError('无法建立第二讲手动演示配置：auto_aim_enabled 字段不唯一')
        (run_dir / 'config.toml').write_text(sim_config)
        sim_env = os.environ.copy()
        sim_env['RM_SIM_RUN_DIR'] = str(run_dir)
        sim_env['DAEDALUS_CAMERA_DIAGNOSTICS'] = '1'
        sim_env['RUST_LOG'] = 'warn,bevy_render::renderer=info,daedalus::camera_diagnostics=info'
        simulator = start('simulator', ['bash', 'scripts/lesson02.sh', 'simulator'], env=sim_env, echo=False)
        training = start('training', [str(ROOT / 'build/training'), 'configs/simulation.yaml'], echo=False)
        log('本次演示已用临时配置关闭自瞄接管：直接按方向键，不需要先按 F5。')
        deadline = time.monotonic() + 50
        while not data['images'] or not data['feedback']:
            spin(0.1)
            check_camera_height()
            if simulator.poll() is not None or training.poll() is not None:
                raise RuntimeError('仿真器或培训程序提前退出，见对应日志')
            if time.monotonic() > deadline:
                raise RuntimeError('50 秒内未收到图像和反馈；检查桌面/GPU/ROS_DOMAIN_ID/QoS')

        log('正在等待场景加载；收到图像不等于已经显示了场地。')
        scene_deadline = time.monotonic() + 25
        last_wait_notice = 0
        while not data['scene_detail']:
            spin(0.1)
            check_camera_height()
            if simulator.poll() is not None or training.poll() is not None:
                raise RuntimeError('等待画面时程序退出，见 simulator.log / training.log')
            if time.monotonic() - last_wait_notice >= 5:
                log('画面仍接近单色，继续等待加载；若窗口是灰色，请不要当成验收通过。')
                last_wait_notice = time.monotonic()
            if time.monotonic() >= scene_deadline:
                save_image(data['last_image'], 'camera-last.png')
                report['checks']['image_has_scene_detail'] = False
                raise RuntimeError('持续收到接近单色的相机图像；已保留灰屏及相机/GPU诊断日志，请先排查显示链路。')

        log('窗口准备：把终端放左半屏；仿真器放右上；camera 窗口放右下。双击标题栏可取消最大化，再拖动边框缩小。')
        log('请始终让终端的 yaw/pitch/bullet_speed 与独立 camera 画面同时可见。')
        prepare_end = time.monotonic() + args.prepare_seconds
        while time.monotonic() < prepare_end:
            spin(min(1., prepare_end - time.monotonic()))
            check_camera_height()
            if simulator.poll() is not None or training.poll() is not None:
                raise RuntimeError('摆放窗口时程序退出，见 simulator.log / training.log')
        # 操纵验收仅统计准备结束之后的数据，避免把摆窗口时的动作算入正式演示。
        data['yaw_travel'] = 0.
        data['pitch_min'] = data['pitch_max'] = data['pitch']
        log('阶段 3：现在开始操纵。请点击【有训练场地／计分板的仿真器窗口】，先轻按 ←/→，再轻按 ↑/↓。')
        log('标题含 camera / view only 的窗口只显示相机图像，不能操纵机器人；不要在终端里按方向键。')
        log('倒计时结束会自动停止并保存视频，期间不用另按录屏快捷键。')
        beginning = dict((key, data[key]) for key in ('images', 'feedback', 'commands'))
        began = time.monotonic()
        end = began + args.seconds
        last_notice = 0
        last_input_hint = 0
        while time.monotonic() < end and training.poll() is None:
            spin(0.1)
            check_camera_height()
            elapsed = time.monotonic() - began
            if elapsed - last_notice >= 10:
                status = (f"{'● 录屏中' if recorder else '诊断中'} | 剩余 {max(0, int(end-time.monotonic())):2} 秒 | "
                          f"yaw={data['last_yaw']:.1f}° pitch={data['pitch']:.1f}° "
                          f"bullet_speed={data['speed']:.0f} m/s | 图像={data['images']} "
                          f"指令={sorted(data['states'])}")
                print(status, flush=True)
                last_notice = elapsed
                if elapsed > 12 and data['yaw_travel'] < 1 and data['pitch_max']-data['pitch_min'] < 1:
                    if elapsed-last_input_hint >= 10:
                        print('\n尚未检测到云台转动：请点击有场地的仿真器，再按方向键。', flush=True)
                        last_input_hint = elapsed
        print(flush=True)
        duration = max(0.001, time.monotonic() - began)
        report['rates_hz'] = {key: (data[key] - beginning[key])/duration for key in beginning}
        report['yaw_travel_deg'] = data['yaw_travel']
        report['pitch_range_deg'] = [data['pitch_min'], data['pitch_max']]
        report['checks']['image_feedback_received'] = data['images'] > 0 and data['feedback'] > 0
        report['checks']['image_has_scene_detail'] = data['scene_detail']
        report['image_detail_stddev'] = data['image_detail_stddev']
        report['checks']['default_commands_stop_only'] = data['states'] == {0}
        report['checks']['yaw_and_pitch_change'] = (data['yaw_travel'] > 1 and
                data['pitch_max'] - data['pitch_min'] > 1)
        report['checks']['main_consumed_frames'] = any(int(n) > 0 for n in
                re.findall(r'frames=(\d+)', (output / 'training.log').read_text()))
        report['checks']['camera_stayed_above_ground'] = (
            camera_health['samples'] > 0 and not camera_health['below_scene'])
        if data['last_image'] is not None:
            save_image(data['last_image'], 'camera-last.png')

        # 保存课堂要求的 CLI 输出。--no-daemon 避免遗留 ROS CLI 后台进程。
        for name, command in [('nodes', ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '3']),
                ('topics', ['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '3']),
                ('image-qos', ['ros2', 'topic', 'info', '/image_raw', '-v', '--no-daemon']),
                ('command-once', ['ros2', 'topic', 'echo', '/vision_send_data', '--once', '--no-daemon'])]:
            try:
                result = subprocess.run(command, cwd=ROOT, text=True, stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT, timeout=8)
                (output / (name + '.txt')).write_text(result.stdout)
                valid_content = bool(result.stdout.strip())
                if name == 'nodes':
                    valid_content = '/training_io' in result.stdout.splitlines()
                elif name == 'topics':
                    valid_content = all(topic in result.stdout.splitlines() for topic in
                                        ('/image_raw', '/vision_receive_data', '/vision_send_data'))
                report['checks']['cli_' + name] = result.returncode == 0 and valid_content
                log('CLI ' + ' '.join(command) + '：' +
                    ('PASS' if report['checks']['cli_' + name] else 'FAIL'))
                if name in ('nodes', 'topics', 'command-once'):
                    print(result.stdout.rstrip(), flush=True)
                elif name == 'image-qos':
                    log('完整类型与 QoS 原始输出已保存：image-qos.txt')
            except subprocess.TimeoutExpired as error:
                (output / (name + '.txt')).write_text(str(error))
                report['checks']['cli_' + name] = False
        if not report['checks'].get('cli_nodes'):
            # CLI 启动时发现可能不足；独立探针已持续运行，可作为补充证据，不能改写 CLI 的原始结果。
            nodes = sorted(node.get_node_names())
            (output / 'probe-nodes.txt').write_text('\n'.join(nodes) + '\n')
            report['checks']['probe_found_training_node'] = 'training_io' in nodes

        check_camera_height()
        log('阶段 4：向培训程序发送 Ctrl+C，观察最后的停止消息，再关闭仿真器。')
        signal_ns = time.time_ns()
        was_running = training.poll() is None
        exit_code = stop(training)
        spin(0.8)
        report['checks']['clean_training_exit'] = exit_code == 0 and '[IO Close]' in (output / 'training.log').read_text()
        report['checks']['close_stop_received'] = (was_running and data['last_state'] == 0 and
                                                   data['last_output_ns'] >= signal_ns)
        stop(simulator)
        # Close 和退出后的停止消息必须留在真实录屏内，封装视频前展示原始检查结果。
        log('===== 第二讲真实运行结果（视频尚待封装） =====')
        for label, key in [('无数据退出', 'no_data_ctrl_c'), ('图像与反馈', 'image_feedback_received'),
                           ('yaw/pitch 变化', 'yaw_and_pitch_change'), ('主循环消费图像', 'main_consumed_frames'),
                           ('默认输出仅停止', 'default_commands_stop_only'), ('CLI 指令观察', 'cli_command-once'),
                           ('培训程序正常退出', 'clean_training_exit'), ('退出后收到停止消息', 'close_stop_received')]:
            log(label + '：' + ('PASS' if report['checks'].get(key) else 'FAIL'))
        log('请保持此终端可见；6 秒后自动结束录屏。')
        if recorder:
            time.sleep(6)
        if recorder:
            stop(recorder, timeout=15)
            video = output / 'lesson02-demo.webm'
            discovery = subprocess.run(['/usr/bin/gst-discoverer-1.0', str(video)], env=clean_env,
                        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
            (output / 'video-info.txt').write_text(discovery.stdout)
            report['checks']['recorded_video'] = (video.exists() and video.stat().st_size > 10000 and
                                                   discovery.returncode == 0 and 'Duration:' in discovery.stdout)
        else:
            report['checks']['recorded_video'] = False
        if all(report['checks'].values()):
            report['status'] = 'PASS_AUTOMATED'
        report['visual_review'] = '请回看 lesson02-demo.webm，确认训练场景、相机图像与操纵过程正常后再提交。'
    except (Exception, KeyboardInterrupt) as error:
        report['errors'].append(str(error) or '演示被中断')
        log('PARTIAL: ' + report['errors'][-1])
    finally:
        # 先培训程序/仿真器，最后录屏，确保 Close 被保留在视频中。
        for proc in reversed(children):
            if proc is not recorder:
                stop(proc)
        stop(recorder, timeout=15)
        try:
            capture.close()
        except Exception as error:
            report['errors'].append('关闭录屏会话：' + str(error))
        for thread in threads:
            thread.join(timeout=2)
        if node is not None:
            node.destroy_node()
        if ros is not None and ros.ok():
            ros.shutdown()
        for stream in handles:
            stream.close()
        report['camera_height_samples'] = camera_health['samples']
        report['camera_min_height_m'] = camera_health['min_height']
        if camera_health['samples']:
            report['checks']['camera_stayed_above_ground'] = not camera_health['below_scene']
        if report['errors'] or not all(report['checks'].values()):
            report['status'] = 'PARTIAL'
        (output / 'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        print('\n' + ('自动检查通过' if report['status'] == 'PASS_AUTOMATED' else '尚有验收项未通过'), flush=True)
        for error in report['errors']:
            print('原因：' + error, flush=True)
        failed = [name for name, value in report['checks'].items() if not value]
        if failed:
            print('待核验：' + ', '.join(failed), flush=True)
        if recorder and (output / 'lesson02-demo.webm').exists():
            print('录屏文件：' + str(output / 'lesson02-demo.webm'), flush=True)
        print('完整日志和 summary.json：' + str(output), flush=True)
    return 0 if report['status'] == 'PASS_AUTOMATED' else 1


if __name__ == '__main__':
    sys.exit(main())
