tzs_ws 软更新 Bundle
====================

源板 IP  : __SRC_IP__
打包时间 : __TIMESTAMP__
包含包   : __PKG_LIST__

bundle 里装的是什么
--------------------
  devel/lib/<pkg>/                          C++ 节点 (.so + wrapper)
  devel/lib/python3/dist-packages/<pkg>/    Python 节点
  src/<pkg>/   (如有)                       launch / config
  start_gimbal.sh                           systemd 启动脚本

【部署】把 deploy_linux.sh / deploy_windows.bat 放到当前目录再跑
----------------------------------------------------------------

Linux / Mac:
  chmod +x deploy_linux.sh
  ./deploy_linux.sh <target_ip>

Windows (cmd.exe):
  deploy_windows.bat <target_ip>

  仅用 Windows 自带的 tar.exe / ssh.exe / scp.exe，
  Win10 1809+ / Win11 默认即可。脚本会提示密码。

默认会自动停服 → 覆盖文件 → 重启服务。
跳过停服/重启: NO_RESTART=1 ./deploy_linux.sh <target_ip>


【部署后必做的 3 步验证】
-------------------------

  1. 看服务日志:
     ssh root@<target_ip> journalctl -u tzs_ws-multi -n 50

  2. 看 ROS 节点:
     ssh root@<target_ip> rosnode list

  3. 看 topic:
     ssh root@<target_ip> rostopic list


【常见问题】
------------

Q: ssh 一直要求输密码。
A: 配密钥  ssh-copy-id root@<target_ip>
   一次配好以后这台机器再部署同板子就免密了。

Q: 服务起不来, "ExecStart permission denied"。
A: start_gimbal.sh 没 +x 位,手动:
     ssh root@<target_ip> "chmod +x /home/rpdzkj/tzs_ws/start_gimbal.sh"
     ssh root@<target_ip> "systemctl restart tzs_ws-multi tzs_ws-payload"

Q: 节点找不到 .so / "error while loading shared libraries"。
A: 源/目标镜像不一致 (一般是 GLIBC)。在两边分别跑:
     ldd /home/rpdzkj/tzs_ws/devel/lib/<pkg>/*.so | grep 'not found'
   有输出就是镜像不同,无法软更新,得重烧。

Q: 部署脚本说找不到 ssh/scp (Windows)。
A: Settings > Apps > Optional Features > Add > OpenSSH Client
   装好后重开 cmd 再跑。

Q: bundle 里文件时间戳看起来是"未来"。
A: 正常,因为源板时钟比目标板晚。目标板不编译,不影响运行。


【目标板环境前置】
------------------

- 跟源板同一份镜像烧出来的 (CPU/glibc/Python 一致)
- ssh 允许 root 登录 (PermitRootLogin yes 或 prohibit-password + 已配密钥)
- ssh 端口 22 可达
- /home/rpdzkj/tzs_ws 目录已存在
- systemd 服务 tzs_ws-multi / tzs_ws-payload 已注册

如果不是同一份镜像烧出来的,请先让管理员重烧,别直接软更新,
否则可能因为 GLIBC / Python 版本不一致导致服务起不来。