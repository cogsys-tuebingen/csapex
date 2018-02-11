#!/home/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups: ~
  assignments:
    - id: 1
      uuid: ~
    - id: 1
      uuid: csapex::ExampleImageGenerator_0
    - uuid: csapex::tutorial::UserSwitch_0
      id: 1
    - id: 1
      uuid: vision_plugins::GenericImageCombiner_0
    - id: 1
      uuid: csapex::OutputDisplay_0
adapters:
  - state:
      width: 512
      height: 512
    uuid: csapex::OutputDisplay_0
connections:
  - uuid: csapex::ExampleImageGenerator_0:|:out_0
    targets:
      - csapex::tutorial::UserSwitch_0:|:in_0
      - vision_plugins::GenericImageCombiner_0:|:in_0
    types:
      - default
      - default
  - types:
      - default
    targets:
      - csapex::tutorial::UserSwitch_0:|:in_1
    uuid: vision_plugins::GenericImageCombiner_0:|:out_0
  - targets:
      - csapex::OutputDisplay_0:|:in_0
    types:
      - default
    uuid: csapex::tutorial::UserSwitch_0:|:out_0
uuid_map:
  vision_plugins::GenericImageCombiner: 1
  csapex::tutorial::UserSwitch: 1
  csapex::ExampleImageGenerator: 1
  csapex::OutputDisplay: 1
nodes:
  - state:
      persistent_params:
        []
      params:
        {}
    uuid: csapex::ExampleImageGenerator_0
    enabled: true
    minimized: false
    z: 3
    color:
      - -1
      - -1
      - -1
    type: csapex::ExampleImageGenerator
    flipped: false
    label: csapex::ExampleImageGenerator_0
    pos:
      - -700
      - -282
  - pos:
      - 251
      - -311
    color:
      - -1
      - -1
      - -1
    flipped: false
    enabled: true
    uuid: csapex::tutorial::UserSwitch_0
    type: csapex::tutorial::UserSwitch
    z: 2
    label: csapex::tutorial::UserSwitch_0
    state:
      params:
        {}
      persistent_params:
        []
    minimized: false
  - color:
      - -1
      - -1
      - -1
    pos:
      - -487
      - 62
    z: 7
    enabled: true
    minimized: false
    flipped: false
    state:
      persistent_params:
        []
      params:
        input count:
          int: 1
          name: input count
          type: value
        script:
          type: value
          name: script
          string: 255-$1
        input names:
          type: string_list
          list:
            - Image
            - Image
          name: input names
    label: vision_plugins::GenericImageCombiner_0
    type: vision_plugins::GenericImageCombiner
    uuid: vision_plugins::GenericImageCombiner_0
  - uuid: csapex::OutputDisplay_0
    state:
      params:
        {}
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    type: csapex::OutputDisplay
    z: 10
    enabled: true
    pos:
      - 661
      - 75
    flipped: false
    label: csapex::OutputDisplay_0
    minimized: false