#!/localhome/buck/ws/apex/devel/lib/csapex/csapex_node
nodes:
  - state:
      params:
        h:
          int: 180
          name: h
          type: value
        w:
          int: 242
          name: w
          type: value
        text:
          name: text
          string: "Tutorial Series: GUI usage\nTutorial 01: Creating connections\n\nTry connecting the output of the source node to the input of the sink node using drag-and-drop.\n\nEither drag the input onto the output or the other way around."
          type: value
      persistent_params:
        []
    flipped: false
    z: -2147483592
    uuid: csapex::Note_0
    minimized: false
    pos:
      - 465
      - -300
    color:
      - 0
      - 0
      - 0
    type: csapex::Note
    label: prototype
    enabled: true
  - enabled: true
    minimized: false
    color:
      - -1
      - -1
      - -1
    label: Input
    state:
      params:
        {}
      persistent_params:
        []
    flipped: false
    type: csapex::ExampleImageGenerator
    z: 14
    pos:
      - 227
      - -97
    uuid: csapex::ExampleImageGenerator_0
  - z: 18
    label: Output
    pos:
      - 721
      - -97
    type: csapex::OutputDisplay
    uuid: csapex::OutputDisplay_0
    flipped: false
    enabled: true
    state:
      params:
        {}
      persistent_params:
        []
    color:
      - -1
      - -1
      - -1
    minimized: false
  - state:
      params:
        w:
          name: w
          type: value
          int: 335
        text:
          type: value
          name: text
          string: "The node \"Input\" produces a stream of images.\nIt has one output port named \"Image\".\n\nYou can hover over it to see the published image.\n"
        h:
          int: 111
          name: h
          type: value
      persistent_params:
        []
    flipped: false
    enabled: true
    pos:
      - 146
      - 18
    type: csapex::Note
    label: prototype
    uuid: csapex::Note_1
    minimized: false
    color:
      - -1
      - -1
      - -1
    z: -2147483617
  - label: prototype
    pos:
      - 720
      - 298
    color:
      - -1
      - -1
      - -1
    state:
      persistent_params:
        []
      params:
        text:
          type: value
          name: text
          string: "The node \"Output\" consumes a stream of visualizable Messages.\nIt has one input port named \"Message\"."
        h:
          type: value
          name: h
          int: 79
        w:
          int: 302
          name: w
          type: value
    uuid: csapex::Note_2
    flipped: false
    minimized: false
    z: -2147483601
    enabled: true
    type: csapex::Note
adapters:
  - uuid: csapex::OutputDisplay_0
    state:
      width: 265
      height: 247
threads:
  groups: ~
  assignments:
    - uuid: ~
      id: 1
    - id: 1
      uuid: csapex::Note_0
    - id: 1
      uuid: csapex::ExampleImageGenerator_0
    - id: 1
      uuid: csapex::OutputDisplay_0
    - uuid: csapex::Note_1
      id: 1
    - id: 1
      uuid: csapex::Note_2
connections:
  []
uuid_map:
  csapex::ExampleImageGenerator: 1
  csapex::OutputDisplay: 1
  csapex::Note: 3