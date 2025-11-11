# Phanes

MRPP (Multi-Robot Path Planning) / MAPF (Multi-Agent Path Finding) in Grid Maps.
增加機器人狀態機和任務狀態
增加任務數量，即m>n，km進行分配
手搓CA*已經完成  -- 設計了全局時間位移到env中 以及時空表的工程性問題
生成任務 – 要改成生成的時候，生成貨物點和運輸指定位置點 
可視化上-取貨送貨的顔色變化

# TODO:
清除不必要的變量和代碼！
分兩階段規劃-path2pickup pickup2delivery
分階段規劃時候的工程問題 如：分配取貨和送貨目標地點的數據結構，重新適配函數。
