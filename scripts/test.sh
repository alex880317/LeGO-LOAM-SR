#!/bin/bash

# 設定變數
LAUNCH_COMMAND="ros2 launch lego_loam_sr run.launch.py"
BAG_FILE="MulranDataset_Riverside01.bag"
PLAY_COMMAND="rosbag play --duration 20 $BAG_FILE /os1_points:=/velodyne_points"
LOG_FILE="ros2_launch.log"  # 臨時儲存日誌的檔案
ITERATIONS=3  # 修改為需要的執行次數


# 檢查日誌是否包含目標關鍵字
wait_for_log() {
    # 接收參數作為關鍵字
    local LOG_KEYWORD="$1"

    # 找到最新的日誌目錄（排除檔案）
    LOG_DIR=$(find ~/.ros/log/ -mindepth 1 -maxdepth 1 -type d -printf "%T@ %p\n" | sort -n -r | head -1 | awk '{print $2}')
    echo "最新的 ROS 日誌目錄為: $LOG_DIR"

    # 確定要檢查的日誌檔案
    LOG_FILE="$LOG_DIR/launch.log"

    # 等待日誌檔案生成
    MAX_RETRIES=10
    RETRY_COUNT=0
    while [ ! -f "$LOG_FILE" ]; do
        if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
            echo "未找到日誌檔案 $LOG_FILE。請確認主程式是否正確啟動。"
            return 1
        fi
        echo "等待日誌檔案生成..."
        sleep 1
        RETRY_COUNT=$((RETRY_COUNT + 1))
    done

    # 檢查日誌檔案是否存在
    if [ -f "$LOG_FILE" ]; then
        echo "正在檢視日誌檔案 $LOG_FILE，等待關鍵字 \"$LOG_KEYWORD\" 出現..."
    else
        echo "未找到日誌檔案 $LOG_FILE。請確認主程式是否正確啟動。"
        return 1  # 返回錯誤碼
    fi

    # 監控日誌直到找到關鍵字
    while true; do
        if grep -q "$LOG_KEYWORD" "$LOG_FILE"; then
            echo "日誌關鍵字 \"$LOG_KEYWORD\" 已出現！主程式啟動完成。"
            break
        fi
        sleep 1
    done
}


# 檢查 topic 是否有數據持續更新
check_topic_update() {
    local TOPIC=$1    # 第一個參數為 ROS topic 名稱
    local TIMEOUT=$2  # 第二個參數為超時秒數

    local LAST_MESSAGE_TIME=$(date +%s)

    echo "正在監控 ROS topic: $TOPIC，超時設定為 $TIMEOUT 秒..."

    while true; do
        local CURRENT_TIME=$(date +%s)

        # 使用 rostopic echo 檢查 topic 是否有新數據
        # 確保在 ROS 1 環境中執行
        source /opt/ros/noetic/setup.bash
        # rostopic echo -n 1 $TOPIC > /dev/null 2>&1
        RATE=$(timeout 5 rostopic hz $TOPIC 2>&1 | sed -n '2p' | awk '{print $3}')
        # 判斷頻率是否為有效數值
        if [[ "$RATE" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
            # 更新最後收到消息的時間
            LAST_MESSAGE_TIME=$CURRENT_TIME
            echo "Topic $TOPIC 有數據更新。"
            echo "Topic $TOPIC 發布頻率: $RATE Hz"
        else
            echo "Topic $TOPIC 無數據或頻率無效（可能已結束）。"
        fi

        # 如果超過 TIMEOUT 秒沒有新數據，認為播放已結束
        if [ $((CURRENT_TIME - LAST_MESSAGE_TIME)) -gt $TIMEOUT ]; then
            echo "Topic $TOPIC 超過 $TIMEOUT 秒無數據，認為 rosbag 播放已結束。"
            break
        fi

    done
}


check_tf_clear() {
    local TF_FRAME=$1 # 第一個參數為要檢查的 TF
    local TIMEOUT=$2  # 第二個參數為超時秒數

    local START_TIME=$(date +%s)

    echo "正在檢查 TF $TF_FRAME 是否已不存在。"
    echo "超時設定為 $TIMEOUT 秒..."

    while true; do
        local CURRENT_TIME=$(date +%s)

        # 檢查 TF 是否仍然存在
        TF_EXISTS=$(ros2 topic echo -n 1 /tf | grep -q "$TF_FRAME" && echo "yes" || echo "no")

        if [[ "$TF_EXISTS" == "no" ]]; then
            echo "TF $TF_FRAME 已不存在。"
            break
        fi

        if [ $((CURRENT_TIME - START_TIME)) -gt $TIMEOUT ]; then
            echo "超時 $TIMEOUT 秒，TF $TF_FRAME 仍然存在，退出檢查。"
            break
        fi

        echo "TF $TF_FRAME 狀態: $TF_EXISTS"
        sleep 1
    done
}



## main
for ((i=1; i<=ITERATIONS; i++)); do
    echo "第 $i 次執行開始..."
    
    # 設定自動化變數
    export ROS_ENV=2  # 自動選擇 ROS2 galactic 環境
    # 啟動 ROS 主程式，將日誌輸出到檔案
    echo "啟動主程式: $LAUNCH_COMMAND"
    # gnome-terminal -- bash -c "source ~/.bashrc && source /opt/ros/galactic/setup.bash && cd ~/colcon_ws && source install/setup.bash && $LAUNCH_COMMAND > $LOG_FILE 2>&1; exec bash" 
    gnome-terminal -- bash -c "unset ROS_DISTRO && unset CMAKE_PREFIX_PATH && unset LD_LIBRARY_PATH && source /opt/ros/galactic/setup.bash && cd ~/colcon_ws && source install/setup.bash && $LAUNCH_COMMAND > $LOG_FILE 2>&1; exec bash"


    # 等待主程式啟動完成
    LOG_KEYWORD="Spinning until killed publishing transform from 'map' to 'camera_init'"  # 目標日誌關鍵字
    wait_for_log "$LOG_KEYWORD"

    # 設定自動化變數
    export ROS_ENV=1  # 自動選擇 ROS1 noetic 環境
    # 啟動 ROS bag 播放
    echo "播放 ROS bag 檔案: $PLAY_COMMAND"
    gnome-terminal -- bash -c "source ~/.bashrc && source /opt/ros/noetic/setup.bash && cd ~/Downloads && $PLAY_COMMAND; " 
    
    # 等待 bag 播放完成
    # 調用函數檢查 topic 是否更新
    check_topic_update "/velodyne_points" 3
    echo "ROS bag 播放完成，關閉主程式..."

    # 結束所有與主程式相關的進程
    pkill -f "ros2 launch lego_loam_sr run.launch.py"

    # # 獲取 ros2 launch 主程式的 PID
    # MAIN_PID=$(pgrep -f "ros2 launch lego_loam_sr run.launch.py")

    # # 如果找到進程，向其發送 SIGINT
    # if [ -n "$MAIN_PID" ]; then
    #     echo "向進程 $MAIN_PID 發送 SIGINT 信號以模擬 Ctrl+C"
    #     kill -SIGINT "$MAIN_PID"
    # else
    #     echo "未找到與 ros2 launch lego_loam_sr run.launch.py 相關的進程。"
    # fi
    
    
    # 確保完全關閉
    # LOG_KEYWORD="process has finished cleanly"  # 目標日誌關鍵字
    # wait_for_log "$LOG_KEYWORD"
    check_tf_clear "/tf" 5
    echo "主程式已關閉。"
done

echo "所有執行已完成。"
