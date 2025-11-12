#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage 6: 集成主脚本 - FIXED VERSION
改进：
1. 正确的后台床加热等待
2. 更好的ROS初始化
3. 完善的错误处理
4. 详细的调试信息
"""

import sys
import os
import argparse
import time
import logging
from datetime import datetime

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 确保能导入模块
try:
    from gcode_sender_251111 import GCodeSender, PrinterState
    logger.info("[IMPORT] ✓ 已导入 GCodeSender")
except ImportError as e:
    logger.error(f"[IMPORT] ✗ 无法导入 GCodeSender: {e}")
    logger.error("请确保 gcode_sender_251111.py 在同一目录")
    sys.exit(1)

try:
    import rospy  # type: ignore
    ROS_AVAILABLE = True
    logger.info("[IMPORT] ✓ ROS 可用")
except ImportError:
    ROS_AVAILABLE = False
    logger.warning("[IMPORT] ⚠ 未找到 ROS")

if ROS_AVAILABLE:
    try:
        from robot_speed_controller_251111 import PrinterSpeedController
        logger.info("[IMPORT] ✓ 已导入 PrinterSpeedController")
    except ImportError as e:
        logger.error(f"[IMPORT] ✗ 无法导入 PrinterSpeedController: {e}")
        logger.error("请确保 robot_speed_controller_251111.py 在同一目录")
        sys.exit(1)


class Stage6IntegratedSystem:
    """Stage 6 集成系统 - FIXED版本"""
    
    def __init__(self, octoprint_url, api_key, speed_source='odom',
                 odom_topic='/odom', cmd_vel_topic='/cmd_vel',
                 speed_threshold=0.04):
        """初始化集成系统"""
        
        if not ROS_AVAILABLE:
            logger.error("[INIT] ✗ 需要 ROS 环境")
            sys.exit(1)
        
        try:
            rospy.init_node('stage6_printer_controller', anonymous=True)
            logger.info("[INIT] ✓ ROS 节点已初始化")
        except rospy.exceptions.ROSException as e:
            logger.error(f"[INIT] ✗ ROS 初始化失败: {e}")
            sys.exit(1)
        
        self.octoprint_url = octoprint_url
        self.api_key = api_key
        self.speed_source = speed_source
        self.odom_topic = odom_topic
        self.cmd_vel_topic = cmd_vel_topic
        self.speed_threshold = speed_threshold
        
        # 初始化打印机控制器
        logger.info("[INIT] 初始化打印机控制器...")
        try:
            self.sender = GCodeSender(octoprint_url, api_key)
            logger.info("[INIT] ✓ 打印机控制器已初始化")
        except Exception as e:
            logger.error(f"[INIT] ✗ 打印机控制器初始化失败: {e}")
            sys.exit(1)
        
        # 初始化速度控制器
        logger.info("[INIT] 初始化速度控制器...")
        try:
            self.controller = PrinterSpeedController(
                self.sender,
                speed_threshold=speed_threshold,
                speed_source=speed_source,
                odom_topic=odom_topic,
                cmd_vel_topic=cmd_vel_topic
            )
            logger.info("[INIT] ✓ 速度控制器已初始化")
        except Exception as e:
            logger.error(f"[INIT] ✗ 速度控制器初始化失败: {e}")
            logger.error("请检查 ROS 话题设置")
            sys.exit(1)
        
        self.current_file = None
        logger.info("[INIT] ✓ 系统初始化完成")
    
    def start_printing_with_speed_control(self, file_path, wait_for_bed=True):
        """启动文件打印并启用速度控制"""
        
        logger.info(f"\n[START] 启动打印: {file_path}")
        
        if not os.path.exists(file_path):
            logger.error(f"[START] ✗ 文件不存在: {file_path}")
            return False
        
        # 初始化日志
        if not self.controller.init_logging():
            logger.warning("[START] ⚠ 日志初始化失败")
        
        # 启动打印（后台等待床加热，不阻塞）
        logger.info("[START] 启动文件打印...")
        if not self.sender.start_file_print(file_path):
            logger.error("[START] ✗ 无法启动打印")
            return False
        
        self.current_file = file_path
        logger.info("[START] ✓ 打印线程已启动")
        
        # 等待床加热完成（如果需要）
        if wait_for_bed:
            logger.info("[START] 等待床加热完成...")
            timeout = 60  # 最多等待60秒
            start_time = time.time()
            
            while not self.sender.is_bed_heat_complete() and (time.time() - start_time) < timeout:
                bed_temp_status = self.sender.check_temperatures_safe()
                bed_data = bed_temp_status.get('bed', {})
                current_temp = bed_data.get('actual', 0)
                target_temp = bed_data.get('target', 0)
                
                logger.info(f"[START] 床温度: {current_temp:.1f}°C / {target_temp:.1f}°C")
                time.sleep(2)
            
            if self.sender.is_bed_heat_complete():
                logger.info("[START] ✓ 床加热完成")
            else:
                logger.warning("[START] ⚠ 床加热超时，继续进行")
        
        # 启动速度控制
        logger.info("[START] 启动速度控制...")
        if not self.controller.start_control():
            logger.error("[START] ✗ 无法启动速度控制")
            return False
        
        logger.info("[START] ✓ 打印和速度控制已启动")
        return True
    
    def stop(self):
        """停止所有操作"""
        logger.info("\n[STOP] 正在停止所有操作...")
        
        if self.controller.control_enabled:
            self.controller.stop_control()
        
        if self.sender.state != PrinterState.IDLE:
            self.sender.stop()
        
        logger.info("[STOP] ✓ 已停止")
    
    def get_system_status(self) -> dict:
        """获取系统状态"""
        controller_status = self.controller.get_status()
        sender_progress = self.sender.get_progress()
        
        return {
            'timestamp': datetime.now().isoformat(),
            'printer_state': sender_progress['state'],
            'printer_progress': f"{sender_progress['progress_percent']:.1f}%",
            'robot_speed': f"{controller_status['current_robot_speed']:.4f} m/s",
            'speed_threshold': f"{controller_status['speed_threshold']:.4f} m/s",
            'printer_paused': controller_status['printer_paused'],
            'speed_monitor_alive': controller_status['speed_monitor_alive'],
            'pause_count': controller_status['pause_count'],
            'resume_count': controller_status['resume_count'],
            'current_file': self.current_file,
            'log_file': self.controller.log_file,
            'bed_heat_complete': sender_progress['bed_heat_complete'],
        }
    
    def interactive_shell(self):
        """交互式命令行"""
        
        print("\n" + "="*80)
        print("Stage 6: 机器人速度 + 打印机集成控制系统 v2.0 (FIXED)")
        print("="*80)
        print("\n系统配置:")
        print(f"  OctoPrint: {self.octoprint_url}")
        print(f"  速度源: {self.speed_source}")
        print(f"  里程计话题: {self.odom_topic}")
        print(f"  速度命令话题: {self.cmd_vel_topic}")
        print(f"  速度阈值: {self.speed_threshold:.4f} m/s")
        print("\n💡 关键命令:")
        print("  print <file>         - 启动打印 (自动启用速度控制)")
        print("  status               - 显示系统状态")
        print("  stop                 - 停止打印")
        print("  threshold <value>    - 设置速度阈值 (m/s)")
        print("\n📝 其他命令:")
        print("  pause                - 手动暂停打印")
        print("  resume               - 手动恢复打印")
        print("  debounce <time>      - 设置防抖时间 (s)")
        print("  logs                 - 显示日志文件位置")
        print("  help                 - 显示帮助信息")
        print("  quit                 - 退出")
        print("="*80 + "\n")
        
        try:
            while True:
                try:
                    cmd_input = input(">>> ").strip()
                    if not cmd_input:
                        continue
                    
                    parts = cmd_input.split(' ', 1)
                    command = parts[0].lower()
                    arg = parts[1] if len(parts) > 1 else None
                    
                    if command == 'quit':
                        logger.info("[CMD] 正在退出...")
                        break
                    
                    elif command == 'print':
                        if arg and os.path.exists(arg):
                            self.start_printing_with_speed_control(arg, wait_for_bed=True)
                        else:
                            print("❌ 请提供有效的文件路径")
                    
                    elif command == 'status':
                        status = self.get_system_status()
                        print("\n📊 系统状态:")
                        for key, value in sorted(status.items()):
                            print(f"  {key}: {value}")
                        print()
                    
                    elif command == 'pause':
                        if self.sender.pause():
                            print("✓ 已暂停")
                        else:
                            print("❌ 暂停失败")
                    
                    elif command == 'resume':
                        if self.sender.resume():
                            print("✓ 已恢复")
                        else:
                            print("❌ 恢复失败")
                    
                    elif command == 'stop':
                        self.stop()
                        print("✓ 已停止")
                    
                    elif command == 'threshold':
                        if arg:
                            try:
                                threshold = float(arg)
                                self.controller.set_speed_threshold(threshold)
                                print(f"✓ 速度阈值已设置为 {threshold:.4f} m/s")
                            except ValueError:
                                print("❌ 请输入有效的数值")
                        else:
                            print("❌ 请提供速度阈值")
                    
                    elif command == 'debounce':
                        if arg:
                            try:
                                debounce = float(arg)
                                self.controller.set_debounce_time(debounce)
                                print(f"✓ 防抖时间已设置为 {debounce:.2f} s")
                            except ValueError:
                                print("❌ 请输入有效的数值")
                        else:
                            print("❌ 请提供防抖时间")
                    
                    elif command == 'logs':
                        if self.controller.log_file:
                            print(f"📄 日志文件: {self.controller.log_file}")
                        else:
                            print("❌ 日志未初始化")
                    
                    elif command == 'help':
                        print("\n📖 帮助信息:")
                        print("  print <file>        - 打开文件并开始打印")
                        print("  status              - 显示当前系统状态")
                        print("  threshold <value>   - 调整速度触发阈值（默认0.04 m/s）")
                        print("  debounce <time>     - 调整防抖时间以避免频繁切换（默认0.5 s）")
                        print("  使用命令时需要输入完整路径，例如: print /home/user/model.gcode\n")
                    
                    else:
                        print(f"❌ 未知命令: {command}，输入 'help' 获取帮助")
                        
                except KeyboardInterrupt:
                    print("\n正在退出...")
                    break
                except Exception as e:
                    print(f"❌ 错误: {e}")
                    logger.exception("命令执行异常")
        
        finally:
            self.stop()
            logger.info("程序已退出")


def main():
    """主函数"""
    
    parser = argparse.ArgumentParser(
        description='Stage 6: 机器人速度 + 打印机集成控制系统'
    )
    parser.add_argument('--url', default='http://octopi.local',
                       help='OctoPrint URL (default: http://octopi.local)')
    parser.add_argument('--api-key', default='HDPJciX6JWnEwFr3R4xTc3rKVcqG63N4vRRLDWhOe7I',
                       help='OctoPrint API Key')
    parser.add_argument('--speed-source', default='cmd_vel', choices=['odom', 'cmd_vel'],
                       help='Speed source: odom (Odometry) or cmd_vel (Velocity Command)')
    parser.add_argument('--odom-topic', default='/kmriiwa/base/state/odom',
                       help='Odometry topic (default: /odom)')
    parser.add_argument('--cmd-vel-topic', default='/kmriiwa/base/command/cmd_vel',
                       help='Velocity command topic (default: /kmriiwa/base/command/cmd_vel)')
    parser.add_argument('--threshold', type=float, default=0.04,
                       help='Speed threshold in m/s (default: 0.04)')
    parser.add_argument('--print-file', default=None,
                       help='G-code file to print (optional, can also use command line)')
    
    args = parser.parse_args()
    
    logger.info("="*80)
    logger.info("Stage 6 集成系统启动")
    logger.info("="*80)
    
    try:
        # 创建集成系统
        system = Stage6IntegratedSystem(
            octoprint_url=args.url,
            api_key=args.api_key,
            speed_source=args.speed_source,
            odom_topic=args.odom_topic,
            cmd_vel_topic=args.cmd_vel_topic,
            speed_threshold=args.threshold
        )
        
        # 如果提供了文件，直接启动打印
        if args.print_file:
            logger.info(f"启动文件打印: {args.print_file}")
            system.start_printing_with_speed_control(args.print_file, wait_for_bed=True)
        
        # 启动交互式命令行
        system.interactive_shell()
        
    except Exception as e:
        logger.error(f"致命错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    if not ROS_AVAILABLE:
        print("❌ 错误: Stage 6 需要在 ROS 环境中运行")
        print("请确保已安装 ROS 并已执行: source /opt/ros/*/setup.bash")
        sys.exit(1)
    
    main()