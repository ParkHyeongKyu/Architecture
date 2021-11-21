# Archi_DMA
POSTECH Architecture DMA

-DMA가 제대로 동작했는지 확인하는법
memory wave를 추가해서 index 200~211까지 값이 제대로 들어갔는지 확인하면 됩니다.

external_device의 데이터는 메모리의 c8~d3까지 저장됩니다.

external_device.v에서 44번째 라인의 count_clk == 16b'20에 원하는 값을 집어넣어서
원하는 특정 사이클에 interrupt가 생기도록 조절 할 수 있습니다.

또한 device_data의 값들을 바꿔서 원하는 데이터를 넣어볼 수 있습니다.
