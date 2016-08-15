#!/home/buck/ws/apex/devel//lib/csapex/csapex_node
threads:
  assignments:
    - id: 1
      uuid: ~
    - id: 1
      uuid: csapex::ExampleImageGenerator_0
    - uuid: csapex::tutorial::UserSwitch_0
      id: 1
    - uuid: vision_plugins::GenericImageCombiner_0
      id: 1
    - uuid: csapex::OutputDisplay_0
      id: 1
    - uuid: csapex::TextDisplay_0
      id: 1
  groups: ~
uuid_map:
  csapex::ExampleImageGenerator: 1
  csapex::OutputDisplay: 1
  csapex::TextDisplay: 1
  csapex::tutorial::UserSwitch: 1
  vision_plugins::GenericImageCombiner: 1
nodes:
  - uuid: csapex::ExampleImageGenerator_0
    label: csapex::ExampleImageGenerator_0
    type: csapex::ExampleImageGenerator
    pos:
      - -700
      - -282
    color:
      - -1
      - -1
      - -1
    z: 13
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::tutorial::UserSwitch_0
    label: csapex::tutorial::UserSwitch_0
    type: csapex::tutorial::UserSwitch
    pos:
      - 251
      - -311
    color:
      - -1
      - -1
      - -1
    z: 2
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        selector:
          type: set
          name: selector
          txt: FIRST
          int: 0
      persistent_params:
        []
  - type: vision_plugins::GenericImageCombiner
    uuid: vision_plugins::GenericImageCombiner_0
    label: vision_plugins::GenericImageCombiner_0
    pos:
      - -487
      - 62
    color:
      - -1
      - -1
      - -1
    z: 14
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        input count:
          type: value
          name: input count
          int: 1
        input names:
          type: string_list
          name: input names
          list:
            - Image
            - Image
        script:
          type: value
          name: script
          string: 255-$1
      persistent_params:
        []
  - type: csapex::OutputDisplay
    uuid: csapex::OutputDisplay_0
    label: csapex::OutputDisplay_0
    pos:
      - 661
      - 75
    color:
      - -1
      - -1
      - -1
    z: 15
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - type: csapex::TextDisplay
    uuid: csapex::TextDisplay_0
    label: csapex::TextDisplay_0
    pos:
      - 289
      - 327
    color:
      - -1
      - -1
      - -1
    z: 12
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
connections:
  - uuid: csapex::tutorial::UserSwitch_0:|:out_1
    targets:
      - csapex::TextDisplay_0:|:in_0
    types:
      - default
  - uuid: csapex::tutorial::UserSwitch_0:|:out_0
    targets:
      - csapex::OutputDisplay_0:|:in_0
    types:
      - default
  - uuid: vision_plugins::GenericImageCombiner_0:|:out_0
    targets:
      - csapex::tutorial::UserSwitch_0:|:in_1
    types:
      - default
  - uuid: csapex::ExampleImageGenerator_0:|:out_0
    targets:
      - csapex::tutorial::UserSwitch_0:|:in_0
      - vision_plugins::GenericImageCombiner_0:|:in_0
    types:
      - default
      - default
adapters:
  - uuid: csapex::OutputDisplay_0
    state:
      width: 512
      height: 512