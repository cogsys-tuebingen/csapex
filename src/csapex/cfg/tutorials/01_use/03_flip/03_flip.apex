#!/localhome/buck/ws/apex/devel/lib/csapex/csapex_node
threads:
  groups: ~
  assignments:
    - uuid: csapex::OutputDisplay_1
      id: 1
    - uuid: ~
      id: 1
    - uuid: csapex::Note_0
      id: 1
    - uuid: csapex::ExampleImageGenerator_0
      id: 1
    - uuid: csapex::BoxBlur_0
      id: 1
uuid_map:
  csapex::BoxBlur: 1
  csapex::ExampleImageGenerator: 1
  csapex::Note: 3
  csapex::OutputDisplay: 2
nodes:
  - type: csapex::Note
    uuid: csapex::Note_0
    label: prototype
    pos:
      - 576
      - -81
    color:
      - 0
      - 0
      - 0
    z: -2147483545
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        h:
          type: value
          name: h
          int: 308
        text:
          type: value
          name: text
          string: "Tutorial Series: GUI usage\nTutorial 03: Flipping nodes\n\nSometimes the flow of data is better visualized from right to left.\n\nTry flipping the sides of the node 'Blur' by right-clicking it and selecing 'flip sides'"
        w:
          type: value
          name: w
          int: 236
      persistent_params:
        []
  - pos:
      - 255
      - -109
    type: csapex::ExampleImageGenerator
    color:
      - -1
      - -1
      - -1
    z: 25
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
    label: Input
    uuid: csapex::ExampleImageGenerator_0
  - uuid: csapex::OutputDisplay_1
    type: csapex::OutputDisplay
    label: Display
    pos:
      - 240
      - 166
    color:
      - -1
      - -1
      - -1
    z: 33
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        {}
      persistent_params:
        []
  - uuid: csapex::BoxBlur_0
    type: csapex::BoxBlur
    label: Blur
    pos:
      - 233
      - 22
    color:
      - 255
      - 0
      - 0
    z: 35
    minimized: false
    enabled: true
    flipped: false
    state:
      params:
        kernel:
          type: range
          name: kernel
          int: 129
          min: 1
          max: 255
          step: 2
      persistent_params:
        []
connections:
  - uuid: csapex::BoxBlur_0:|:out_0
    targets:
      - csapex::OutputDisplay_1:|:in_0
    types:
      - default
  - uuid: csapex::ExampleImageGenerator_0:|:out_0
    targets:
      - csapex::BoxBlur_0:|:in_0
    types:
      - default
adapters:
  - uuid: csapex::OutputDisplay_1
    state:
      width: 100
      height: 100