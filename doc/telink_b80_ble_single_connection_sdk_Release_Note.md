# V3.4.1.2 (Public release)

### Features
   * Normalizes the naming of content related to private schemas
   
### BREAKING CHANGES
   * N/A

### CodeSize
   * Flash:
      - Disable deepsleep retention: 39.7k Byte
      - Enable deepsleep retention: 40.5k Byte
   * RAM:
      - Disable deepsleep retention: 12.5k Byte
      - Enable deepsleep retention: 13.7k Byte
   * The values above are obtained by compiling 8208_ble_sample with default configuration. 

   * Flash:
      - Disable deepsleep retention: 39.6k Byte
      - Enable deepsleep retention: 40.0k Byte
   * RAM:
      - Disable deepsleep retention: 10.5k Byte
      - Enable deepsleep retention: 11.2k Byte
   * The values above are obtained by compiling 8208_ble_sample with calling APIs of saving RAM
   
### Features
   * 规范化与私有模式相关内容的命名
   
### BREAKING CHANGES
   * N/A

### CodeSize
   * Flash:
      - 关闭deepsleep retention: 39.7k Byte
      - 打开deepsleep retention: 40.5k Byte
   * RAM:
      - 关闭deepsleep retention: 12.5k Byte
      - 打开deepsleep retention: 13.7k Byte
   * 以上大小使用默认配置编译8208_ble_sample得到

   * Flash:
      - 关闭deepsleep retention: 39.6k Byte
      - 打开deepsleep retention: 40.0k Byte
   * RAM:
      - 关闭deepsleep retention: 10.5k Byte
      - 打开deepsleep retention: 11.2k Byte
   * 以上大小时是调用节省ram的API后得到

## V3.4.1.1_Patch_0002

### Features
   * N/A

### BREAKING CHANGES
   * N/A
   
### Bug Fixes
   * Fix the bug that the value of T_IFS is inaccurate when 48M is used.
	
### CodeSize
   * Flash:
      - Disable deepsleep retention: 39.7k Byte
      - Enable deepsleep retention: 40.6k Byte
   * RAM:
      - Disable deepsleep retention: 12.4k Byte
      - Enable deepsleep retention: 13.7k Byte
   * The values above are obtained by compiling 8208_ble_sample with default configuration
   
   * Flash:
      - Disable deepsleep retention: 39.7k Byte
      - Enable deepsleep retention: 40.0k Byte
   * RAM:
      - Disable deepsleep retention: 10.4k Byte
      - Enable deepsleep retention: 11.2k Byte
   * The values above are obtained by compiling 8208_ble_sample with calling APIs of saving RAM
   
   
   
### Features
   * N/A

### BREAKING CHANGES
   * N/A
   
### Bug Fixes
   * 修复使用48M时T_IFS值不准确的问题。
	
### CodeSize
   * Flash:
      - 关闭deepsleep retention: 39.7k Byte
      - 打开deepsleep retention: 40.6k Byte
   * RAM:
      - 关闭deepsleep retention: 12.4k Byte
      - 打开deepsleep retention: 13.7k Byte
   * 以上大小使用默认配置编译8208_ble_sample得到

   * Flash:
      - 关闭deepsleep retention: 39.7k Byte
      - 打开deepsleep retention: 40.0k Byte
   * RAM:
      - 关闭deepsleep retention: 10.4k Byte
      - 打开deepsleep retention: 11.2k Byte
    * 以上大小使用调用节省RAM的API编译8208_ble_sample得到

## V3.4.1.1_Patch_0001

### Features
   * SDK version：telink_b80_ble_single_connection_sdk_v3.4.1.1_patch_0001
   * Add API which is used to set whether to continue current ADV_event when recieveing SACN_REQ
   * Add long sleep function

### BREAKING CHANGES
   * N/A
   
### Bug Fixes
   * Solve the problem of crystal oscillator stability flag failure
	
### CodeSize
   * Flash:
      - Disable deepsleep retention: 39.6k Byte
      - Enable deepsleep retention: 40.5k Byte
   * RAM:
      - Disable deepsleep retention: 12.4k Byte
      - Enable deepsleep retention: 13.7k Byte
   * The values above are obtained by compiling 8208_ble_sample with default configuration
   
   * Flash:
      - Disable deepsleep retention: 39.6k Byte
      - Enable deepsleep retention: 39.9k Byte
   * RAM:
      - Disable deepsleep retention: 10.4k Byte
      - Enable deepsleep retention: 11.2k Byte
   * The values above are obtained by compiling 8208_ble_sample with calling APIs of saving RAM
   
   
   
### Features
   * SDK 版本：telink_b80_ble_single_connection_sdk_v3.4.1.1_patch_0001
   * 增加API,当在广播事件里收到SCAN_REQ时，可以选择是否继续当前广播事件
   * 增加长睡眠函数

### BREAKING CHANGES
   * N/A
   
### Bug Fixes
   * 解决晶振起振失败的问题。
	
### CodeSize
   * Flash:
      - 关闭deepsleep retention: 39.6k Byte
      - 打开deepsleep retention: 40.5k Byte
   * RAM:
      - 关闭deepsleep retention: 12.4k Byte
      - 打开deepsleep retention: 13.7k Byte
   * 以上大小使用默认配置编译8208_ble_sample得到

   * Flash:
      - 关闭deepsleep retention: 39.6k Byte
      - 打开deepsleep retention: 39.9k Byte
   * RAM:
      - 关闭deepsleep retention: 10.4k Byte
      - 打开deepsleep retention: 11.2k Byte
    * 以上大小使用调用节省RAM的API编译8208_ble_sample得到
	

## V3.4.1.1

### Dependency Updates
   * telink_b85m_driver_sdk_V1.5.0
### Features
   * Support passive scanning in slave connection
   * Support passive scanning in advertising
   * Add APIs of saving RAM, but these APIs increase power consumption when called
   * Fix the issue that the pointer to packet is null when callback of BLT_EV_FLAG_CONNECT is triggered
   * Add Puya Flash
   * Add version information at the end of bin file.
   
### BREAKING CHANGES
   * N/A

### CodeSize
   * Flash:
      - Disable deepsleep retention: 39.4k Byte
      - Enable deepsleep retention: 40.3k Byte
   * RAM:
      - Disable deepsleep retention: 12.4k Byte
      - Enable deepsleep retention: 13.6k Byte
   * The values above are obtained by compiling 8208_ble_sample with default configuration. 

   * Flash:
      - Disable deepsleep retention: 39.3k Byte
      - Enable deepsleep retention: 39.7k Byte
   * RAM:
      - Disable deepsleep retention: 10.4k Byte
      - Enable deepsleep retention: 11.2k Byte
   * The values above are obtained by compiling 8208_ble_sample with calling APIs of saving RAM
### Dependency Updates
   * telink_b85m_driver_sdk_V1.5.0
### Features
   * 支持在连接中被动扫描
   * 支持在广播中被动扫描
   * 增加api来节省RAM，在调用时会增加平均功耗
   * 修复了触发BLT_EV_FLAG_CONNECT事件回调时指向数据包的指针为空的问题
   * 适配Puya Flash
   * bin文件末尾增加版本信息
   
### BREAKING CHANGES
   * N/A

### CodeSize
   * Flash:
      - 关闭deepsleep retention: 39.4k Byte
      - 打开deepsleep retention: 40.3k Byte
   * RAM:
      - 关闭deepsleep retention: 12.4k Byte
      - 打开deepsleep retention: 13.6k Byte
   * 以上大小使用默认配置编译8208_ble_sample得到

   * Flash:
      - 关闭deepsleep retention: 39.3k Byte
      - 打开deepsleep retention: 39.7k Byte
   * RAM:
      - 关闭deepsleep retention: 10.4k Byte
      - 打开deepsleep retention: 11.2k Byte
   * 以上大小时是调用节省ram的API后得到