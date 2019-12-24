@rem
@rem Licensed to the Apache Software Foundation (ASF) under one
@rem or more contributor license agreements.  See the NOTICE file
@rem distributed with this work for additional information
@rem regarding copyright ownership.  The ASF licenses this file
@rem to you under the Apache License, Version 2.0 (the
@rem "License"); you may not use this file except in compliance
@rem with the License.  You may obtain a copy of the License at
@rem
@rem  http://www.apache.org/licenses/LICENSE-2.0
@rem
@rem Unless required by applicable law or agreed to in writing,
@rem software distributed under the License is distributed on an
@rem "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
@rem KIND, either express or implied.  See the License for the
@rem specific language governing permissions and limitations
@rem under the License.
@rem

@rem Execute a shell with a script of the same name and .sh extension

@bash "%~dp0%~n0.sh"


@rem set WSLENV=CORE_PATH/u:BSP_PATH/u:IMAGE_SLOT/u:FEATURES/u:EXTRA_JTAG_CMD/u:MFG_IMAGE/u:FLASH_OFFSET/u:BOOT_LOADER/u
@rem @bash /mnt/c/dev/wProto-MyNewt/wProtoMyNewt/hw/bsp/wyresRevB-BasedStm32l152discovery/stm32l152discovery_download.sh