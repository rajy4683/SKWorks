node {
  name: "code_input"
  op: "Placeholder"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
        dim {
          size: 32
        }
      }
    }
  }
}
node {
  name: "image_input"
  op: "Placeholder"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
        dim {
          size: 192
        }
        dim {
          size: 256
        }
        dim {
          size: 1
        }
      }
    }
  }
}
node {
  name: "zeros"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 32
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "stack"
  op: "Pack"
  input: "zeros"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "add"
  op: "Add"
  input: "code_input"
  input: "stack"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "transpose"
  op: "Transpose"
  input: "image_input"
  input: "transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\007\000\000\000\007\000\000\000\001\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.23035022616386414
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 7
        }
        dim {
          size: 7
        }
        dim {
          size: 1
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_0/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_0/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_0/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_0/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\003\000\000\000\003\000\000\000\003\000\000\000\003\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/MirrorPad"
  op: "MirrorPad"
  input: "transpose"
  input: "CNN/img_decompose/img_unet/enc/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_0/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_0/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_0/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_0/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000@\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.06718548387289047
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 64
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_1/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_1/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_1/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_1/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_0/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_1/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_1/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_1/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_1/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000@\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.06718548387289047
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 64
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_2/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_2/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_2/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_2/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_1/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_2/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_2/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_2/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_2/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_3/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_3/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_3/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_3/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_2/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_3/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_3/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_3/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_3/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_4/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_4/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_4/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_4/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_3/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_4/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_4/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_4/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_4/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.033592741936445236
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 256
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_5/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_5/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_5/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_5/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_4/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_5/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_5/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_5/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_5/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.033592741936445236
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 256
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_6/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_6/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_6/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_6/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_5/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_6/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_6/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_6/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_6/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/enc/conv_7/biases"
  input: "CNN/img_decompose/img_unet/enc/conv_7/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/enc/conv_7/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/enc/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/enc/conv_7/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/enc/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/enc/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_6/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_7/MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/enc/conv_7/Conv2D"
  input: "CNN/img_decompose/img_unet/enc/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/enc/conv_7/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_7/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_0/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_0/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_0/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_0/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/enc/conv_7/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_0/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_0/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_0/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_0/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_1/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_1/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_1/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_1/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_0/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_1/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_1/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_1/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample/transpose"
  op: "Transpose"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/upsample/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/img_decompose/img_unet/dec/upsample/transpose"
  input: "CNN/img_decompose/img_unet/dec/upsample/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample/transpose_1"
  op: "Transpose"
  input: "CNN/img_decompose/img_unet/dec/upsample/ResizeBilinear"
  input: "CNN/img_decompose/img_unet/dec/upsample/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_2/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_2/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_2/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_2/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/upsample/transpose_1"
  input: "CNN/img_decompose/img_unet/dec/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_2/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_2/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_2/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_2/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/concat/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/concat/concat"
  op: "ConcatV2"
  input: "CNN/img_decompose/img_unet/dec/conv_2/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_5/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/concat/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_3/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_3/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_3/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_3/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/concat/concat"
  input: "CNN/img_decompose/img_unet/dec/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_3/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_3/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_3/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_1/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_1/transpose"
  op: "Transpose"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/upsample_1/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_1/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "0\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_1/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/img_decompose/img_unet/dec/upsample_1/transpose"
  input: "CNN/img_decompose/img_unet/dec/upsample_1/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_1/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_1/transpose_1"
  op: "Transpose"
  input: "CNN/img_decompose/img_unet/dec/upsample_1/ResizeBilinear"
  input: "CNN/img_decompose/img_unet/dec/upsample_1/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.033592741936445236
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 256
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_4/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_4/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_4/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_4/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/upsample_1/transpose_1"
  input: "CNN/img_decompose/img_unet/dec/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_4/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_4/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_4/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_4/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/concat_1/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/concat_1/concat"
  op: "ConcatV2"
  input: "CNN/img_decompose/img_unet/dec/conv_4/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_3/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/concat_1/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.033592741936445236
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 256
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_5/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_5/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_5/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_5/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/concat_1/concat"
  input: "CNN/img_decompose/img_unet/dec/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_5/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_5/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_5/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_2/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_2/transpose"
  op: "Transpose"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/upsample_2/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_2/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_2/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/img_decompose/img_unet/dec/upsample_2/transpose"
  input: "CNN/img_decompose/img_unet/dec/upsample_2/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_2/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/upsample_2/transpose_1"
  op: "Transpose"
  input: "CNN/img_decompose/img_unet/dec/upsample_2/ResizeBilinear"
  input: "CNN/img_decompose/img_unet/dec/upsample_2/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_6/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_6/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_6/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_6/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/upsample_2/transpose_1"
  input: "CNN/img_decompose/img_unet/dec/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_6/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_6/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_6/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_6/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/concat_2/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/concat_2/concat"
  op: "ConcatV2"
  input: "CNN/img_decompose/img_unet/dec/conv_6/LeakyRelu"
  input: "CNN/img_decompose/img_unet/enc/conv_1/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/concat_2/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/mul"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/biases/Assign"
  op: "Assign"
  input: "CNN/img_decompose/img_unet/dec/conv_7/biases"
  input: "CNN/img_decompose/img_unet/dec/conv_7/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/biases/read"
  op: "Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_7/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/img_decompose/img_unet/dec/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/img_decompose/img_unet/dec/conv_7/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/img_decompose/img_unet/dec/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/img_decompose/img_unet/dec/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/concat_2/concat"
  input: "CNN/img_decompose/img_unet/dec/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/Conv2D"
  op: "Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_7/MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/BiasAdd"
  op: "BiasAdd"
  input: "CNN/img_decompose/img_unet/dec/conv_7/Conv2D"
  input: "CNN/img_decompose/img_unet/dec/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_7/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/strided_slice/stack"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/strided_slice/stack_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/strided_slice/stack_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/strided_slice"
  op: "StridedSlice"
  input: "transpose"
  input: "CNN/strided_slice/stack"
  input: "CNN/strided_slice/stack_1"
  input: "CNN/strided_slice/stack_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\007\000\000\000\007\000\000\000\001\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.23035022616386414
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 7
        }
        dim {
          size: 7
        }
        dim {
          size: 1
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_0/kernel"
  input: "CNN/dpt_enc/conv_0/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_0/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_0/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_0/biases"
  input: "CNN/dpt_enc/conv_0/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_0/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_0/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\003\000\000\000\003\000\000\000\003\000\000\000\003\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/MirrorPad"
  op: "MirrorPad"
  input: "transpose"
  input: "CNN/dpt_enc/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_0/MirrorPad"
  input: "CNN/dpt_enc/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_0/Conv2D"
  input: "CNN/dpt_enc/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_0/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_0/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dpt_enc/concat/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/concat/concat"
  op: "ConcatV2"
  input: "CNN/dpt_enc/conv_0/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  input: "CNN/dpt_enc/concat/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_1/kernel"
  input: "CNN/dpt_enc/conv_1/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_1/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_1/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_1/biases"
  input: "CNN/dpt_enc/conv_1/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_1/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_1/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_enc/concat/concat"
  input: "CNN/dpt_enc/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_1/MirrorPad"
  input: "CNN/dpt_enc/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_1/Conv2D"
  input: "CNN/dpt_enc/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_1/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_1/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000@\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.06718548387289047
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 64
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_2/kernel"
  input: "CNN/dpt_enc/conv_2/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_2/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_2/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_2/biases"
  input: "CNN/dpt_enc/conv_2/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_2/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_2/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_enc/conv_1/LeakyRelu"
  input: "CNN/dpt_enc/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_2/MirrorPad"
  input: "CNN/dpt_enc/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_2/Conv2D"
  input: "CNN/dpt_enc/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_2/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_2/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dpt_enc/concat_1/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/concat_1/concat"
  op: "ConcatV2"
  input: "CNN/dpt_enc/conv_2/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  input: "CNN/dpt_enc/concat_1/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.033592741936445236
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 256
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_3/kernel"
  input: "CNN/dpt_enc/conv_3/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_3/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_3/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_3/biases"
  input: "CNN/dpt_enc/conv_3/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_3/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_3/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_enc/concat_1/concat"
  input: "CNN/dpt_enc/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_3/MirrorPad"
  input: "CNN/dpt_enc/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_3/Conv2D"
  input: "CNN/dpt_enc/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_3/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_3/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_4/kernel"
  input: "CNN/dpt_enc/conv_4/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_4/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_4/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_4/biases"
  input: "CNN/dpt_enc/conv_4/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_4/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_4/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_enc/conv_3/LeakyRelu"
  input: "CNN/dpt_enc/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_4/MirrorPad"
  input: "CNN/dpt_enc/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_4/Conv2D"
  input: "CNN/dpt_enc/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_4/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_4/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dpt_enc/concat_2/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/concat_2/concat"
  op: "ConcatV2"
  input: "CNN/dpt_enc/conv_4/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  input: "CNN/dpt_enc/concat_2/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_5/kernel"
  input: "CNN/dpt_enc/conv_5/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_5/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_5/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_5/biases"
  input: "CNN/dpt_enc/conv_5/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_5/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_5/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_enc/concat_2/concat"
  input: "CNN/dpt_enc/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_5/MirrorPad"
  input: "CNN/dpt_enc/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_5/Conv2D"
  input: "CNN/dpt_enc/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_5/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_5/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.033592741936445236
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 256
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_6/kernel"
  input: "CNN/dpt_enc/conv_6/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_6/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_6/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_6/biases"
  input: "CNN/dpt_enc/conv_6/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_6/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_6/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_enc/conv_5/LeakyRelu"
  input: "CNN/dpt_enc/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_6/MirrorPad"
  input: "CNN/dpt_enc/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_6/Conv2D"
  input: "CNN/dpt_enc/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_6/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_6/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dpt_enc/concat_3/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/concat_3/concat"
  op: "ConcatV2"
  input: "CNN/dpt_enc/conv_6/LeakyRelu"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  input: "CNN/dpt_enc/concat_3/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\004\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.016796370968222618
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 1024
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_7/kernel"
  input: "CNN/dpt_enc/conv_7/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_7/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_7/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_enc/conv_7/biases"
  input: "CNN/dpt_enc/conv_7/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/biases/read"
  op: "Identity"
  input: "CNN/dpt_enc/conv_7/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_enc/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_enc/conv_7/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_enc/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_enc/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_enc/concat_3/concat"
  input: "CNN/dpt_enc/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_enc/conv_7/MirrorPad"
  input: "CNN/dpt_enc/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_enc/conv_7/Conv2D"
  input: "CNN/dpt_enc/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_enc/conv_7/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dpt_enc/conv_7/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/flatten/Reshape/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\377\377\377\377"
      }
    }
  }
}
node {
  name: "CNN/flatten/Reshape"
  op: "Reshape"
  input: "CNN/dpt_enc/conv_7/LeakyRelu"
  input: "CNN/flatten/Reshape/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\000\200\001\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.005142817273736
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/mul"
  input: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 98304
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Assign"
  op: "Assign"
  input: "CNN/dense_enc/dense_0/weights"
  input: "CNN/dense_enc/dense_0/weights/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/read"
  op: "Identity"
  input: "CNN/dense_enc/dense_0/weights"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dense_enc/dense_0/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/weights/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dense_enc/dense_0/weights/Regularizer/l2_regularizer/scale"
  input: "CNN/dense_enc/dense_0/weights/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/biases/Assign"
  op: "Assign"
  input: "CNN/dense_enc/dense_0/biases"
  input: "CNN/dense_enc/dense_0/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/biases/read"
  op: "Identity"
  input: "CNN/dense_enc/dense_0/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dense_enc/dense_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dense_enc/dense_0/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dense_enc/dense_0/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/MatMul"
  op: "MatMul"
  input: "CNN/flatten/Reshape"
  input: "CNN/dense_enc/dense_0/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/Add"
  op: "Add"
  input: "CNN/dense_enc/dense_0/MatMul"
  input: "CNN/dense_enc/dense_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dense_enc/dense_0/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dense_enc/dense_0/Add"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\000\002\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.071260966360569
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/mul"
  input: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Assign"
  op: "Assign"
  input: "CNN/dense_enc/dense_1/weights"
  input: "CNN/dense_enc/dense_1/weights/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/read"
  op: "Identity"
  input: "CNN/dense_enc/dense_1/weights"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dense_enc/dense_1/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/weights/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dense_enc/dense_1/weights/Regularizer/l2_regularizer/scale"
  input: "CNN/dense_enc/dense_1/weights/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/weights"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/biases/Assign"
  op: "Assign"
  input: "CNN/dense_enc/dense_1/biases"
  input: "CNN/dense_enc/dense_1/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/biases/read"
  op: "Identity"
  input: "CNN/dense_enc/dense_1/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dense_enc/dense_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dense_enc/dense_1/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dense_enc/dense_1/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dense_enc/dense_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/MatMul"
  op: "MatMul"
  input: "CNN/dense_enc/dense_0/LeakyRelu"
  input: "CNN/dense_enc/dense_1/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/Add"
  op: "Add"
  input: "CNN/dense_enc/dense_1/MatMul"
  input: "CNN/dense_enc/dense_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dense_enc/dense_1/LeakyRelu"
  op: "LeakyRelu"
  input: "CNN/dense_enc/dense_1/Add"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "alpha"
    value {
      f: 0.20000000298023224
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\000\002\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.071260966360569
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/gen_code/mean/weights/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/gen_code/mean/weights/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/gen_code/mean/weights/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/gen_code/mean/weights/Initializer/truncated_normal/mul"
  input: "CNN/gen_code/mean/weights/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
        dim {
          size: 32
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Assign"
  op: "Assign"
  input: "CNN/gen_code/mean/weights"
  input: "CNN/gen_code/mean/weights/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/read"
  op: "Identity"
  input: "CNN/gen_code/mean/weights"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/gen_code/mean/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/weights/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/gen_code/mean/weights/Regularizer/l2_regularizer/scale"
  input: "CNN/gen_code/mean/weights/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 32
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 32
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/gen_code/mean/biases/Assign"
  op: "Assign"
  input: "CNN/gen_code/mean/biases"
  input: "CNN/gen_code/mean/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gen_code/mean/biases/read"
  op: "Identity"
  input: "CNN/gen_code/mean/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/biases"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/gen_code/mean/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/biases"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/gen_code/mean/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/gen_code/mean/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/mean/biases"
      }
    }
  }
}
node {
  name: "CNN/gen_code/mean/MatMul"
  op: "MatMul"
  input: "CNN/dense_enc/dense_1/LeakyRelu"
  input: "CNN/gen_code/mean/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gen_code/mean/Add"
  op: "Add"
  input: "CNN/gen_code/mean/MatMul"
  input: "CNN/gen_code/mean/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\000\002\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.071260966360569
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/gen_code/std/weights/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/gen_code/std/weights/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/gen_code/std/weights/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/gen_code/std/weights/Initializer/truncated_normal/mul"
  input: "CNN/gen_code/std/weights/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
        dim {
          size: 32
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Assign"
  op: "Assign"
  input: "CNN/gen_code/std/weights"
  input: "CNN/gen_code/std/weights/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/read"
  op: "Identity"
  input: "CNN/gen_code/std/weights"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/gen_code/std/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/weights/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/gen_code/std/weights/Regularizer/l2_regularizer/scale"
  input: "CNN/gen_code/std/weights/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/weights"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 32
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 32
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/gen_code/std/biases/Assign"
  op: "Assign"
  input: "CNN/gen_code/std/biases"
  input: "CNN/gen_code/std/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gen_code/std/biases/read"
  op: "Identity"
  input: "CNN/gen_code/std/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/biases"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/gen_code/std/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/biases"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/gen_code/std/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/gen_code/std/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gen_code/std/biases"
      }
    }
  }
}
node {
  name: "CNN/gen_code/std/MatMul"
  op: "MatMul"
  input: "CNN/dense_enc/dense_1/LeakyRelu"
  input: "CNN/gen_code/std/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gen_code/std/Add"
  op: "Add"
  input: "CNN/gen_code/std/MatMul"
  input: "CNN/gen_code/std/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gen_code/mul/x"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gen_code/mul"
  op: "Mul"
  input: "CNN/gen_code/mul/x"
  input: "CNN/gen_code/mean/Add"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: " \000\000\000\000\200\001\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.285043865442276
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 32
        }
        dim {
          size: 98304
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dense/weights"
  input: "CNN/dpt_decode/dense/weights/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/read"
  op: "Identity"
  input: "CNN/dpt_decode/dense/weights"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dense/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/weights/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dense/weights/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dense/weights/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/weights"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 98304
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 98304
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dense/biases"
  input: "CNN/dpt_decode/dense/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dense/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dense/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dense/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dense/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dense/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/MatMul"
  op: "MatMul"
  input: "add"
  input: "CNN/dpt_decode/dense/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/Add"
  op: "Add"
  input: "CNN/dpt_decode/dense/MatMul"
  input: "CNN/dpt_decode/dense/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dense/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dense/Add"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/reshape/Reshape/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/reshape/Reshape"
  op: "Reshape"
  input: "CNN/dpt_decode/dense/Identity"
  input: "CNN/dpt_decode/reshape/Reshape/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/reshape/Reshape"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_0/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_0/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_0/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_0/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec/conv_0/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec/mul"
  input: "CNN/dpt_decode/dpt_dec/concat/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\006\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.013714178465306759
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 1536
        }
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 512
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 512
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/concat/concat"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_1/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_1/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_1/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample/transpose"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec/conv_1/Identity"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample/transpose_1"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.023753654211759567
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 512
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose_1"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_2/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_2/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_2/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/mul_1"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_2/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat_1/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat_1/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec/conv_2/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec/mul_1"
  input: "CNN/dpt_decode/dpt_dec/concat_1/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\003\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.019394777715206146
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 768
        }
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 256
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 256
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_3/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/concat_1/concat"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_3/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_3/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_3/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_1/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_1/transpose"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec/conv_3/Identity"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_1/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "0\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_1/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_1/transpose_1"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.033592741936445236
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 256
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_4/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose_1"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_4/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_4/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_4/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/mul_2"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_4/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat_2/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat_2/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec/conv_4/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec/mul_2"
  input: "CNN/dpt_decode/dpt_dec/concat_2/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.027428356930613518
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 384
        }
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 128
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 128
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_5/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/concat_2/concat"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_5/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_5/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_5/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_2/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_2/transpose"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec/conv_5/Identity"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_2/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_2/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/upsample_2/transpose_1"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_6/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose_1"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_6/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_6/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_6/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/mul_3"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_6/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat_3/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/concat_3/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec/conv_6/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec/mul_3"
  input: "CNN/dpt_decode/dpt_dec/concat_3/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\300\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.03878955543041229
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 192
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 64
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_dec/conv_7/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/concat_3/concat"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec/conv_7/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec/conv_7/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec/conv_7/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\007\000\000\000\007\000\000\000\001\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.23035022616386414
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 7
        }
        dim {
          size: 7
        }
        dim {
          size: 1
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 1
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose/output_shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose"
  op: "Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose/output_shape"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/read"
  input: "CNN/dpt_decode/dpt_dec/conv_7/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000@\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.06718548387289047
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 64
        }
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 1
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_7/Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 1
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_pyr/pyr_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_5/Identity"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr/pyr_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/add/y"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/add"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/BiasAdd"
  input: "CNN/dpt_decode/add/y"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/add_1/y"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/add_1"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/BiasAdd"
  input: "CNN/dpt_decode/add_1/y"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/add_2/y"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/add_2"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/BiasAdd"
  input: "CNN/dpt_decode/add_2/y"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\007\000\000\000\007\000\000\000\001\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.23035022616386414
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 7
        }
        dim {
          size: 7
        }
        dim {
          size: 1
        }
        dim {
          size: 64
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 1
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_0/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/conv2d_transpose/output_shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/conv2d_transpose"
  op: "Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/conv2d_transpose/output_shape"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/read"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/conv2d_transpose"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000@\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.06718548387289047
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 64
        }
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 1
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_1/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/shape"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/mean"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/stddev"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.047507308423519135
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  op: "TruncatedNormal"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "seed"
    value {
      i: 0
    }
  }
  attr {
    key: "seed2"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/TruncatedNormal"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/stddev"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal"
  op: "Add"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal/mean"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 3
        }
        dim {
          size: 3
        }
        dim {
          size: 128
        }
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Initializer/truncated_normal"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Initializer/Const"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
          dim {
            size: 1
          }
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
  op: "VariableV2"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "container"
    value {
      s: ""
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "shape"
    value {
      shape {
        dim {
          size: 1
        }
      }
    }
  }
  attr {
    key: "shared_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Assign"
  op: "Assign"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Initializer/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "use_locking"
    value {
      b: true
    }
  }
  attr {
    key: "validate_shape"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/read"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Regularizer/l2_regularizer/scale"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Regularizer/l2_regularizer/L2Loss"
  op: "L2Loss"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Regularizer/l2_regularizer"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Regularizer/l2_regularizer/scale"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/Regularizer/l2_regularizer/L2Loss"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/dpt_decode/dpt_std_pyr/pyr_2/biases"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr/pyr_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dense_1/MatMul"
  op: "MatMul"
  input: "CNN/gen_code/mul"
  input: "CNN/dpt_decode/dense/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/dpt_decode/dense_1/Add"
  op: "Add"
  input: "CNN/dpt_decode/dense_1/MatMul"
  input: "CNN/dpt_decode/dense/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dense_1/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dense_1/Add"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/reshape_1/Reshape/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/reshape_1/Reshape"
  op: "Reshape"
  input: "CNN/dpt_decode/dense_1/Identity"
  input: "CNN/dpt_decode/reshape_1/Reshape/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_0/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_0/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/reshape_1/Reshape"
  input: "CNN/dpt_decode/dpt_dec_1/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_0/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_0/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_0/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_0/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_0/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/mul"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec_1/conv_0/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec_1/conv_0/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec_1/mul"
  input: "CNN/dpt_decode/dpt_dec_1/concat/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/concat/concat"
  input: "CNN/dpt_decode/dpt_dec_1/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_1/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_1/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_1/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_1/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample/transpose"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec_1/conv_1/Identity"
  input: "CNN/dpt_decode/dpt_dec_1/upsample/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec_1/upsample/transpose"
  input: "CNN/dpt_decode/dpt_dec_1/upsample/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample/transpose_1"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec_1/upsample/ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec_1/upsample/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/upsample/transpose_1"
  input: "CNN/dpt_decode/dpt_dec_1/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_2/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_2/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_2/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_2/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/mul_1"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec_1/conv_2/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat_1/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat_1/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec_1/conv_2/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec_1/mul_1"
  input: "CNN/dpt_decode/dpt_dec_1/concat_1/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_3/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_3/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/concat_1/concat"
  input: "CNN/dpt_decode/dpt_dec_1/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_3/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_3/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_3/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_3/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_3/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_3/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_3/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec_1/conv_3/Identity"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_1/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "0\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_1/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_1/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose_1"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_1/ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_4/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_4/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_1/transpose_1"
  input: "CNN/dpt_decode/dpt_dec_1/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_4/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_4/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_4/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_4/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_4/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_4/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_4/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/mul_2"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec_1/conv_4/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat_2/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat_2/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec_1/conv_4/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec_1/mul_2"
  input: "CNN/dpt_decode/dpt_dec_1/concat_2/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_5/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_5/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/concat_2/concat"
  input: "CNN/dpt_decode/dpt_dec_1/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_5/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_5/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_5/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_5/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_5/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_5/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_5/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\002\000\000\000\003\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec_1/conv_5/Identity"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_2/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_2/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_2/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\000\000\000\000\003\000\000\000\001\000\000\000\002\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose_1"
  op: "Transpose"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_2/ResizeBilinear"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_6/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_6/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/upsample_2/transpose_1"
  input: "CNN/dpt_decode/dpt_dec_1/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_6/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_6/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_6/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_6/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_6/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_6/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_6/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/mul_3"
  op: "Mul"
  input: "CNN/dpt_decode/dpt_dec_1/conv_6/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat_3/concat/axis"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/concat_3/concat"
  op: "ConcatV2"
  input: "CNN/dpt_decode/dpt_dec_1/conv_6/Identity"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  input: "CNN/dpt_decode/dpt_dec_1/mul_3"
  input: "CNN/dpt_decode/dpt_dec_1/concat_3/concat/axis"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_7/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_7/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/concat_3/concat"
  input: "CNN/dpt_decode/dpt_dec_1/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_7/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_dec_1/conv_7/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_7/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_dec_1/conv_7/Conv2D"
  input: "CNN/dpt_decode/dpt_dec/conv_7/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_dec_1/conv_7/Identity"
  op: "Identity"
  input: "CNN/dpt_decode/dpt_dec_1/conv_7/BiasAdd"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_0/conv2d_transpose/output_shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_0/conv2d_transpose"
  op: "Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_0/conv2d_transpose/output_shape"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/read"
  input: "CNN/dpt_decode/dpt_dec_1/conv_7/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_0/conv2d_transpose"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/conv_7/Identity"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_1/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_1/Conv2D"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/dpt_decode/dpt_dec_1/conv_5/Identity"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_2/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_pyr_1/pyr_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_2/Conv2D"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/add_3/y"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/add_3"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_0/BiasAdd"
  input: "CNN/dpt_decode/add_3/y"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/add_4/y"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/add_4"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_1/BiasAdd"
  input: "CNN/dpt_decode/add_4/y"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/add_5/y"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/add_5"
  op: "Add"
  input: "CNN/dpt_decode/dpt_pyr_1/pyr_2/BiasAdd"
  input: "CNN/dpt_decode/add_5/y"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_0/conv2d_transpose/output_shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_0/conv2d_transpose"
  op: "Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_0/conv2d_transpose/output_shape"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/kernel/read"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_0/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_0/conv2d_transpose"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_0/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_1/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_1/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_1/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_1/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_1/Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_1/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_2/MirrorPad/paddings"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_2/MirrorPad"
  op: "MirrorPad"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_2/Conv2D"
  op: "Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/dpt_decode/dpt_std_pyr_1/pyr_2/BiasAdd"
  op: "BiasAdd"
  input: "CNN/dpt_decode/dpt_std_pyr_1/pyr_2/Conv2D"
  input: "CNN/dpt_decode/dpt_std_pyr/pyr_2/biases/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/Reshape/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\377\377\377\377"
      }
    }
  }
}
node {
  name: "CNN/Reshape"
  op: "Reshape"
  input: "CNN/dpt_decode/add"
  input: "CNN/Reshape/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/strided_slice_1/stack"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_1/stack_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_1/stack_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_1"
  op: "StridedSlice"
  input: "CNN/Reshape"
  input: "CNN/strided_slice_1/stack"
  input: "CNN/strided_slice_1/stack_1"
  input: "CNN/strided_slice_1/stack_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/zeros/shape_as_tensor"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 49152
      }
    }
  }
}
node {
  name: "CNN/zeros/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/zeros"
  op: "Fill"
  input: "CNN/zeros/shape_as_tensor"
  input: "CNN/zeros/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/mul"
  op: "Mul"
  input: "CNN/zeros"
  input: "CNN/strided_slice_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/Sum"
  op: "Sum"
  input: "CNN/mul"
  input: "CNN/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/gradients/grad_ys_0"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gradients/Fill"
  op: "Fill"
  input: "CNN/gradients/Shape"
  input: "CNN/gradients/grad_ys_0"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/Sum_grad/Reshape/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/Sum_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients/Fill"
  input: "CNN/gradients/CNN/Sum_grad/Reshape/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/Sum_grad/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 49152
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/Sum_grad/Tile"
  op: "Tile"
  input: "CNN/gradients/CNN/Sum_grad/Reshape"
  input: "CNN/gradients/CNN/Sum_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tmultiples"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/mul_grad/Mul"
  op: "Mul"
  input: "CNN/gradients/CNN/Sum_grad/Tile"
  input: "CNN/strided_slice_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients/CNN/Sum_grad/Tile"
  input: "CNN/zeros"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/strided_slice_1_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\300\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad"
  op: "StridedSliceGrad"
  input: "CNN/gradients/CNN/strided_slice_1_grad/Shape"
  input: "CNN/strided_slice_1/stack"
  input: "CNN/strided_slice_1/stack_1"
  input: "CNN/strided_slice_1/stack_2"
  input: "CNN/gradients/CNN/mul_grad/Mul_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/gradients/CNN/Reshape_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad"
  input: "CNN/gradients/CNN/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/add_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/add_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/add_grad/BroadcastGradientArgs"
  op: "BroadcastGradientArgs"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Shape"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/add_grad/Sum"
  op: "Sum"
  input: "CNN/gradients/CNN/Reshape_grad/Reshape"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/add_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Sum"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/add_grad/Sum_1"
  op: "Sum"
  input: "CNN/gradients/CNN/Reshape_grad/Reshape"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/BroadcastGradientArgs:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/add_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Sum_1"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\007\000\000\000\007\000\000\000\001\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Reshape"
  input: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Shape"
  input: "CNN/dpt_decode/dpt_dec/conv_7/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Reshape"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  input: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat_3/concat/axis"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/mod"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset:2"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_6/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/AddN"
  op: "AddN"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  input: "CNN/gradients/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad"
  op: "ResizeBilinearGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat_2/concat/axis"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/mod"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:2"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_4/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/AddN_1"
  op: "AddN"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  input: "CNN/gradients/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad"
  op: "ResizeBilinearGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat_1/concat/axis"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/mod"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset:2"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_2/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/AddN_2"
  op: "AddN"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  input: "CNN/gradients/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad"
  op: "ResizeBilinearGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat/concat/axis"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/mod"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset:1"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset:2"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_0/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients/AddN_3"
  op: "AddN"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  input: "CNN/gradients/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\200\001\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\200\001\000"
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 98304
      }
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs"
  op: "BroadcastGradientArgs"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Shape"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum"
  op: "Sum"
  input: "CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Reshape"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_1"
  op: "Sum"
  input: "CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Reshape"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_1"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul"
  op: "MatMul"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape"
  input: "CNN/dpt_decode/dense/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_1"
  op: "MatMul"
  input: "add"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: true
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients/stack_grad/unstack"
  op: "Unpack"
  input: "CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
  attr {
    key: "num"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/Const_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/TensorArray/size"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 32
      }
    }
  }
}
node {
  name: "CNN/TensorArray"
  op: "TensorArrayV3"
  input: "CNN/TensorArray/size"
  attr {
    key: "clear_after_read"
    value {
      b: true
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "dynamic_size"
    value {
      b: false
    }
  }
  attr {
    key: "element_shape"
    value {
      shape {
        unknown_rank: true
      }
    }
  }
  attr {
    key: "identical_element_shapes"
    value {
      b: true
    }
  }
  attr {
    key: "tensor_array_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/while/Enter"
  op: "Enter"
  input: "CNN/Const_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: false
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/Enter_1"
  op: "Enter"
  input: "CNN/TensorArray:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: false
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/Merge"
  op: "Merge"
  input: "CNN/while/Enter"
  input: "CNN/while/NextIteration"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/Merge_1"
  op: "Merge"
  input: "CNN/while/Enter_1"
  input: "CNN/while/NextIteration_1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/Less/y"
  op: "Const"
  input: "^CNN/while/Merge"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 32
      }
    }
  }
}
node {
  name: "CNN/while/Less"
  op: "Less"
  input: "CNN/while/Merge"
  input: "CNN/while/Less/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/LoopCond"
  op: "LoopCond"
  input: "CNN/while/Less"
}
node {
  name: "CNN/while/Switch"
  op: "Switch"
  input: "CNN/while/Merge"
  input: "CNN/while/LoopCond"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/Merge"
      }
    }
  }
}
node {
  name: "CNN/while/Switch_1"
  op: "Switch"
  input: "CNN/while/Merge_1"
  input: "CNN/while/LoopCond"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/Merge_1"
      }
    }
  }
}
node {
  name: "CNN/while/Identity"
  op: "Identity"
  input: "CNN/while/Switch:1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/Identity_1"
  op: "Identity"
  input: "CNN/while/Switch_1:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/add/y"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/add"
  op: "Add"
  input: "CNN/while/Identity"
  input: "CNN/while/add/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/add_1/y"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/add_1"
  op: "Add"
  input: "CNN/while/Identity"
  input: "CNN/while/add_1/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/strided_slice/stack"
  op: "Pack"
  input: "CNN/while/Identity"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/strided_slice/stack_1"
  op: "Pack"
  input: "CNN/while/add_1"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/strided_slice/stack_2"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/strided_slice"
  op: "StridedSlice"
  input: "CNN/while/strided_slice/Enter"
  input: "CNN/while/strided_slice/stack"
  input: "CNN/while/strided_slice/stack_1"
  input: "CNN/while/strided_slice/stack_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/while/strided_slice/Enter"
  op: "Enter"
  input: "CNN/gradients/stack_grad/unstack"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/while/gradients/grad_ys_0"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/while/gradients/Fill"
  op: "Fill"
  input: "CNN/while/gradients/Shape"
  input: "CNN/while/gradients/grad_ys_0"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/while/strided_slice_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 32
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/while/strided_slice_grad/StridedSliceGrad"
  op: "StridedSliceGrad"
  input: "CNN/while/gradients/CNN/while/strided_slice_grad/Shape"
  input: "CNN/while/strided_slice/stack"
  input: "CNN/while/strided_slice/stack_1"
  input: "CNN/while/strided_slice/stack_2"
  input: "CNN/while/gradients/Fill"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/stack_grad/unstack_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/while/strided_slice_grad/StridedSliceGrad"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul"
  op: "MatMul"
  input: "CNN/while/gradients/CNN/gradients/stack_grad/unstack_grad/stack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dense/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul_1"
  op: "MatMul"
  input: "CNN/while/gradients/CNN/gradients/stack_grad/unstack_grad/stack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: true
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul_1/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 98304
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\200\001\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 2
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/add"
  op: "Add"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/add/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/add/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/mod"
  op: "FloorMod"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/add"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape_1"
  op: "Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/mod"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/start"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/delta"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/range"
  op: "Range"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/start"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/delta"
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill/value"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill"
  op: "Fill"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill/value"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/DynamicStitch"
  op: "DynamicStitch"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/range"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/mod"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum/y"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum"
  op: "Maximum"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/DynamicStitch"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/floordiv"
  op: "FloorDiv"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/DynamicStitch"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Tile"
  op: "Tile"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/floordiv"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tmultiples"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dense/Add_grad/Sum_grad/Tile"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\006\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\006\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/AddN"
  op: "AddN"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/AddN"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\006\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\003\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\003\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/AddN_1"
  op: "AddN"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/AddN_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\003\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "0\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\001\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\001\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/AddN_2"
  op: "AddN"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/AddN_2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/transpose"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\300\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\300\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/AddN_3"
  op: "AddN"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Pad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while/gradients/AddN_3"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\300\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN/Enter_1"
  input: "^CNN/while/Identity"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN/Enter_1"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_pyr/pyr_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN/Enter_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/ShapeN:1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "SAME"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 2
        i: 2
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/dpt_pyr/pyr_0/conv2d_transpose_grad/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000\300\000\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Size"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/add"
  op: "Add"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/add/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/add/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/dpt_decode/add_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/mod"
  op: "FloorMod"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/add"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape_1"
  op: "Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/mod"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/range/start"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/range/delta"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/range"
  op: "Range"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/range/start"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Size"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/range/delta"
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Fill/value"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Fill"
  op: "Fill"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Fill/value"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/DynamicStitch"
  op: "DynamicStitch"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/range"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/mod"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Fill"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Maximum/y"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Maximum"
  op: "Maximum"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/DynamicStitch"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Maximum/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/floordiv"
  op: "FloorDiv"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Maximum"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Reshape_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/DynamicStitch"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Tile"
  op: "Tile"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/floordiv"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tmultiples"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/Reshape_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\300\000\000"
      }
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/Reshape_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/dpt_decode/add_grad/Sum_grad/Tile"
  input: "CNN/while/gradients/CNN/gradients/CNN/Reshape_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice"
  op: "StridedSlice"
  input: "CNN/while/gradients/CNN/gradients/CNN/Reshape_grad/Reshape_grad/Reshape"
  input: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice/Enter"
  input: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice/Enter_1"
  input: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice/Enter_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice/Enter"
  op: "Enter"
  input: "CNN/strided_slice_1/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice/Enter_1"
  op: "Enter"
  input: "CNN/strided_slice_1/stack_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice/Enter_2"
  op: "Enter"
  input: "CNN/strided_slice_1/stack_2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice"
  input: "CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/zeros"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul_1"
  op: "Mul"
  input: "CNN/while/gradients/CNN/gradients/CNN/strided_slice_1_grad/StridedSliceGrad_grad/StridedSlice"
  input: "CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients/CNN/Sum_grad/Tile"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/TensorArrayWrite/TensorArrayWriteV3"
  op: "TensorArrayWriteV3"
  input: "CNN/while/TensorArrayWrite/TensorArrayWriteV3/Enter"
  input: "CNN/while/Identity"
  input: "CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul_1"
  input: "CNN/while/Identity_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul_1"
      }
    }
  }
}
node {
  name: "CNN/while/TensorArrayWrite/TensorArrayWriteV3/Enter"
  op: "Enter"
  input: "CNN/TensorArray"
  attr {
    key: "T"
    value {
      type: DT_RESOURCE
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while/gradients/CNN/gradients/CNN/mul_grad/Mul_1_grad/Mul_1"
      }
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while/NextIteration"
  op: "NextIteration"
  input: "CNN/while/add"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/NextIteration_1"
  op: "NextIteration"
  input: "CNN/while/TensorArrayWrite/TensorArrayWriteV3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while/Exit"
  op: "Exit"
  input: "CNN/while/Switch"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while/Exit_1"
  op: "Exit"
  input: "CNN/while/Switch_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/TensorArrayStack/TensorArraySizeV3"
  op: "TensorArraySizeV3"
  input: "CNN/TensorArray"
  input: "CNN/while/Exit_1"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray"
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack/range/start"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack/range/delta"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack/range"
  op: "Range"
  input: "CNN/TensorArrayStack/range/start"
  input: "CNN/TensorArrayStack/TensorArraySizeV3"
  input: "CNN/TensorArrayStack/range/delta"
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray"
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack/TensorArrayGatherV3"
  op: "TensorArrayGatherV3"
  input: "CNN/TensorArray"
  input: "CNN/TensorArrayStack/range"
  input: "CNN/while/Exit_1"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "element_shape"
    value {
      shape {
        dim {
          size: 49152
        }
      }
    }
  }
}
node {
  name: "CNN/transpose/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/transpose"
  op: "Transpose"
  input: "CNN/TensorArrayStack/TensorArrayGatherV3"
  input: "CNN/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/stack"
  op: "Pack"
  input: "CNN/transpose"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/Reshape_1/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\377\377\377\377"
      }
    }
  }
}
node {
  name: "CNN/Reshape_1"
  op: "Reshape"
  input: "CNN/dpt_decode/add_1"
  input: "CNN/Reshape_1/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/strided_slice_2/stack"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_2/stack_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_2/stack_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_2"
  op: "StridedSlice"
  input: "CNN/Reshape_1"
  input: "CNN/strided_slice_2/stack"
  input: "CNN/strided_slice_2/stack_1"
  input: "CNN/strided_slice_2/stack_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/zeros_1/shape_as_tensor"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 12288
      }
    }
  }
}
node {
  name: "CNN/zeros_1/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/zeros_1"
  op: "Fill"
  input: "CNN/zeros_1/shape_as_tensor"
  input: "CNN/zeros_1/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/mul_1"
  op: "Mul"
  input: "CNN/zeros_1"
  input: "CNN/strided_slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/Const_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/Sum_1"
  op: "Sum"
  input: "CNN/mul_1"
  input: "CNN/Const_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_1/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/gradients_1/grad_ys_0"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gradients_1/Fill"
  op: "Fill"
  input: "CNN/gradients_1/Shape"
  input: "CNN/gradients_1/grad_ys_0"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/Sum_1_grad/Reshape/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/Sum_1_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_1/Fill"
  input: "CNN/gradients_1/CNN/Sum_1_grad/Reshape/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/Sum_1_grad/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 12288
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/Sum_1_grad/Tile"
  op: "Tile"
  input: "CNN/gradients_1/CNN/Sum_1_grad/Reshape"
  input: "CNN/gradients_1/CNN/Sum_1_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tmultiples"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/mul_1_grad/Mul"
  op: "Mul"
  input: "CNN/gradients_1/CNN/Sum_1_grad/Tile"
  input: "CNN/strided_slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/mul_1_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients_1/CNN/Sum_1_grad/Tile"
  input: "CNN/zeros_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/strided_slice_2_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\0000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad"
  op: "StridedSliceGrad"
  input: "CNN/gradients_1/CNN/strided_slice_2_grad/Shape"
  input: "CNN/strided_slice_2/stack"
  input: "CNN/strided_slice_2/stack_1"
  input: "CNN/strided_slice_2/stack_2"
  input: "CNN/gradients_1/CNN/mul_1_grad/Mul_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/Reshape_1_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/Reshape_1_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad"
  input: "CNN/gradients_1/CNN/Reshape_1_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/BroadcastGradientArgs"
  op: "BroadcastGradientArgs"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Shape"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum"
  op: "Sum"
  input: "CNN/gradients_1/CNN/Reshape_1_grad/Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_1"
  op: "Sum"
  input: "CNN/gradients_1/CNN/Reshape_1_grad/Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/BroadcastGradientArgs:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_1"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/read"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat_3/concat/axis"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/mod"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset:2"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_6/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/AddN"
  op: "AddN"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  input: "CNN/gradients_1/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad"
  op: "ResizeBilinearGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_2/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat_2/concat/axis"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/mod"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:2"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_4/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/AddN_1"
  op: "AddN"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  input: "CNN/gradients_1/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad"
  op: "ResizeBilinearGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample_1/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat_1/concat/axis"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/mod"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset:2"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_2/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/AddN_2"
  op: "AddN"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  input: "CNN/gradients_1/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad"
  op: "ResizeBilinearGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/dpt_decode/dpt_dec/upsample/transpose/perm"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat/concat/axis"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/mod"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset:1"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset:2"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_0/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_1/AddN_3"
  op: "AddN"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_1/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  input: "CNN/gradients_1/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_1/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\200\001\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\200\001\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 98304
      }
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs"
  op: "BroadcastGradientArgs"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Shape"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum"
  op: "Sum"
  input: "CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_1"
  op: "Sum"
  input: "CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_1"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul"
  op: "MatMul"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape"
  input: "CNN/dpt_decode/dense/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_1"
  op: "MatMul"
  input: "add"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: true
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_1/stack_grad/unstack"
  op: "Unpack"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
  attr {
    key: "num"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/Const_3"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/TensorArray_1/size"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 32
      }
    }
  }
}
node {
  name: "CNN/TensorArray_1"
  op: "TensorArrayV3"
  input: "CNN/TensorArray_1/size"
  attr {
    key: "clear_after_read"
    value {
      b: true
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "dynamic_size"
    value {
      b: false
    }
  }
  attr {
    key: "element_shape"
    value {
      shape {
        unknown_rank: true
      }
    }
  }
  attr {
    key: "identical_element_shapes"
    value {
      b: true
    }
  }
  attr {
    key: "tensor_array_name"
    value {
      s: ""
    }
  }
}
node {
  name: "CNN/while_1/Enter"
  op: "Enter"
  input: "CNN/Const_3"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: false
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/Enter_1"
  op: "Enter"
  input: "CNN/TensorArray_1:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: false
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/Merge"
  op: "Merge"
  input: "CNN/while_1/Enter"
  input: "CNN/while_1/NextIteration"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/Merge_1"
  op: "Merge"
  input: "CNN/while_1/Enter_1"
  input: "CNN/while_1/NextIteration_1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/Less/y"
  op: "Const"
  input: "^CNN/while_1/Merge"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 32
      }
    }
  }
}
node {
  name: "CNN/while_1/Less"
  op: "Less"
  input: "CNN/while_1/Merge"
  input: "CNN/while_1/Less/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/LoopCond"
  op: "LoopCond"
  input: "CNN/while_1/Less"
}
node {
  name: "CNN/while_1/Switch"
  op: "Switch"
  input: "CNN/while_1/Merge"
  input: "CNN/while_1/LoopCond"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/Merge"
      }
    }
  }
}
node {
  name: "CNN/while_1/Switch_1"
  op: "Switch"
  input: "CNN/while_1/Merge_1"
  input: "CNN/while_1/LoopCond"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/Merge_1"
      }
    }
  }
}
node {
  name: "CNN/while_1/Identity"
  op: "Identity"
  input: "CNN/while_1/Switch:1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/Identity_1"
  op: "Identity"
  input: "CNN/while_1/Switch_1:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/add/y"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/add"
  op: "Add"
  input: "CNN/while_1/Identity"
  input: "CNN/while_1/add/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/add_1/y"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/add_1"
  op: "Add"
  input: "CNN/while_1/Identity"
  input: "CNN/while_1/add_1/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/strided_slice/stack"
  op: "Pack"
  input: "CNN/while_1/Identity"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/strided_slice/stack_1"
  op: "Pack"
  input: "CNN/while_1/add_1"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/strided_slice/stack_2"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/strided_slice"
  op: "StridedSlice"
  input: "CNN/while_1/strided_slice/Enter"
  input: "CNN/while_1/strided_slice/stack"
  input: "CNN/while_1/strided_slice/stack_1"
  input: "CNN/while_1/strided_slice/stack_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/while_1/strided_slice/Enter"
  op: "Enter"
  input: "CNN/gradients_1/stack_grad/unstack"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/grad_ys_0"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/Fill"
  op: "Fill"
  input: "CNN/while_1/gradients/Shape"
  input: "CNN/while_1/gradients/grad_ys_0"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/while_1/strided_slice_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 32
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/while_1/strided_slice_grad/StridedSliceGrad"
  op: "StridedSliceGrad"
  input: "CNN/while_1/gradients/CNN/while_1/strided_slice_grad/Shape"
  input: "CNN/while_1/strided_slice/stack"
  input: "CNN/while_1/strided_slice/stack_1"
  input: "CNN/while_1/strided_slice/stack_2"
  input: "CNN/while_1/gradients/Fill"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/stack_grad/unstack_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/while_1/strided_slice_grad/StridedSliceGrad"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul"
  op: "MatMul"
  input: "CNN/while_1/gradients/CNN/gradients_1/stack_grad/unstack_grad/stack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: false
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dense/weights/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul_1"
  op: "MatMul"
  input: "CNN/while_1/gradients/CNN/gradients_1/stack_grad/unstack_grad/stack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "transpose_a"
    value {
      b: true
    }
  }
  attr {
    key: "transpose_b"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul_1/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 98304
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/MatMul_grad/MatMul_grad/MatMul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\200\001\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 2
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/add"
  op: "Add"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/add/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/add/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/mod"
  op: "FloorMod"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/add"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape_1"
  op: "Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/mod"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/start"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/delta"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/range"
  op: "Range"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/start"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Size"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/range/delta"
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill/value"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill"
  op: "Fill"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill/value"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/DynamicStitch"
  op: "DynamicStitch"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/range"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/mod"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Fill"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum/y"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum"
  op: "Maximum"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/DynamicStitch"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/floordiv"
  op: "FloorDiv"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Maximum"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Reshape_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/DynamicStitch"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Tile"
  op: "Tile"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/floordiv"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tmultiples"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dense/Add_grad/Sum_grad/Tile"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/reshape/Reshape_grad/Reshape_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_0/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/AddN_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_0/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\006\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_1/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_0/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\002\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\006\000\000\014\000\000\000\020\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_grad/Mul_grad/Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/AddN"
  op: "AddN"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/AddN"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\006\000\000\000\002\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while_1/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_grad/transpose_grad/transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while_1/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample/transpose_1_grad/transpose_grad/transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\002\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/AddN_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_2/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\003\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_3/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_2/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\001\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\000\003\000\000\030\000\000\000 \000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_1_grad/Mul_grad/Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/AddN_1"
  op: "AddN"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_1/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/AddN_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_3/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\003\000\000\000\001\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_3/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while_1/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_3/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "0\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_grad/transpose_grad/transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while_1/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_1/transpose_1_grad/transpose_grad/transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\000\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/AddN_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\001\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\001\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_grad/Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/AddN_2"
  op: "AddN"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/AddN_2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\001\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while_1/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  op: "ResizeBilinear"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_grad/transpose_grad/transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "align_corners"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation"
  op: "InvertPermutation"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  input: "^CNN/while_1/Identity"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/transpose"
  op: "Transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/ResizeBilinear_grad/ResizeBilinearGrad_grad/ResizeBilinear"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/InvertPermutation"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/upsample_2/transpose_1_grad/transpose_grad/transpose"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_6/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\200\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_6/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\300\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/img_decompose/img_unet/dec/conv_7/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul_1"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_6/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Rank"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000@\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack/1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack"
  op: "Pack"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Rank"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack/1"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/ConcatOffset:2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape_1"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\300\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub_1"
  op: "Sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape/Enter"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/sub_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat/axis"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat"
  op: "ConcatV2"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Reshape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat/axis"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Pad"
  op: "Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/mul_3_grad/Mul_grad/Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/concat"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/AddN_3"
  op: "AddN"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Pad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_2_grad/Pad"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/concat_3/concat_grad/Slice_grad/Pad"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/AddN_3"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_7/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000\300\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_dec/conv_7/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  op: "MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_dec/conv_7/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\003\000\000\000\003\000\000\000@\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2DBackpropFilter/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  op: "Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/MirrorPad_grad/MirrorPadGrad_grad/MirrorPad"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D/Enter"
  op: "Enter"
  input: "CNN/dpt_decode/dpt_pyr/pyr_1/kernel/read"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/dpt_pyr/pyr_1/Conv2D_grad/Conv2DBackpropInput_grad/Conv2D"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000`\000\000\000\200\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Size"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/add"
  op: "Add"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/add/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/add/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/dpt_decode/add_1_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/mod"
  op: "FloorMod"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/add"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Size"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape_1"
  op: "Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/mod"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/range/start"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/range/delta"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/range"
  op: "Range"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/range/start"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Size"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/range/delta"
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Fill/value"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Fill"
  op: "Fill"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Fill/value"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/DynamicStitch"
  op: "DynamicStitch"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/range"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/mod"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Fill"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Maximum/y"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Maximum"
  op: "Maximum"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/DynamicStitch"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Maximum/y"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/floordiv"
  op: "FloorDiv"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Maximum"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Shape"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Reshape_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/DynamicStitch"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Tile"
  op: "Tile"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/floordiv"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tmultiples"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/Reshape_1_grad/Reshape_grad/Shape"
  op: "Const"
  input: "^CNN/while_1/Identity"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\0000\000\000"
      }
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/Reshape_1_grad/Reshape_grad/Reshape"
  op: "Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/dpt_decode/add_1_grad/Sum_grad/Tile"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/Reshape_1_grad/Reshape_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice"
  op: "StridedSlice"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/Reshape_1_grad/Reshape_grad/Reshape"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice/Enter"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice/Enter_1"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice/Enter_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice/Enter"
  op: "Enter"
  input: "CNN/strided_slice_2/stack"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice/Enter_1"
  op: "Enter"
  input: "CNN/strided_slice_2/stack_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice/Enter_2"
  op: "Enter"
  input: "CNN/strided_slice_2/stack_2"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul/Enter"
  op: "Enter"
  input: "CNN/zeros_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul_1"
  op: "Mul"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/strided_slice_2_grad/StridedSliceGrad_grad/StridedSlice"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul_1/Enter"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul_1/Enter"
  op: "Enter"
  input: "CNN/gradients_1/CNN/Sum_1_grad/Tile"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/TensorArrayWrite/TensorArrayWriteV3"
  op: "TensorArrayWriteV3"
  input: "CNN/while_1/TensorArrayWrite/TensorArrayWriteV3/Enter"
  input: "CNN/while_1/Identity"
  input: "CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul_1"
  input: "CNN/while_1/Identity_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul_1"
      }
    }
  }
}
node {
  name: "CNN/while_1/TensorArrayWrite/TensorArrayWriteV3/Enter"
  op: "Enter"
  input: "CNN/TensorArray_1"
  attr {
    key: "T"
    value {
      type: DT_RESOURCE
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/while_1/gradients/CNN/gradients_1/CNN/mul_1_grad/Mul_1_grad/Mul_1"
      }
    }
  }
  attr {
    key: "frame_name"
    value {
      s: "CNN/while_1/while_context"
    }
  }
  attr {
    key: "is_constant"
    value {
      b: true
    }
  }
  attr {
    key: "parallel_iterations"
    value {
      i: 10
    }
  }
}
node {
  name: "CNN/while_1/NextIteration"
  op: "NextIteration"
  input: "CNN/while_1/add"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/NextIteration_1"
  op: "NextIteration"
  input: "CNN/while_1/TensorArrayWrite/TensorArrayWriteV3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/while_1/Exit"
  op: "Exit"
  input: "CNN/while_1/Switch"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/while_1/Exit_1"
  op: "Exit"
  input: "CNN/while_1/Switch_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/TensorArrayStack_1/TensorArraySizeV3"
  op: "TensorArraySizeV3"
  input: "CNN/TensorArray_1"
  input: "CNN/while_1/Exit_1"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray_1"
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack_1/range/start"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray_1"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack_1/range/delta"
  op: "Const"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray_1"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack_1/range"
  op: "Range"
  input: "CNN/TensorArrayStack_1/range/start"
  input: "CNN/TensorArrayStack_1/TensorArraySizeV3"
  input: "CNN/TensorArrayStack_1/range/delta"
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray_1"
      }
    }
  }
}
node {
  name: "CNN/TensorArrayStack_1/TensorArrayGatherV3"
  op: "TensorArrayGatherV3"
  input: "CNN/TensorArray_1"
  input: "CNN/TensorArrayStack_1/range"
  input: "CNN/while_1/Exit_1"
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/TensorArray_1"
      }
    }
  }
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "element_shape"
    value {
      shape {
        dim {
          size: 12288
        }
      }
    }
  }
}
node {
  name: "CNN/transpose_1/perm"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/transpose_1"
  op: "Transpose"
  input: "CNN/TensorArrayStack_1/TensorArrayGatherV3"
  input: "CNN/transpose_1/perm"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tperm"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/stack_1"
  op: "Pack"
  input: "CNN/transpose_1"
  attr {
    key: "N"
    value {
      i: 1
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "axis"
    value {
      i: 0
    }
  }
}
node {
  name: "CNN/Reshape_2/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\377\377\377\377"
      }
    }
  }
}
node {
  name: "CNN/Reshape_2"
  op: "Reshape"
  input: "CNN/dpt_decode/add_2"
  input: "CNN/Reshape_2/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/strided_slice_3/stack"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\000\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_3/stack_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_3/stack_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/strided_slice_3"
  op: "StridedSlice"
  input: "CNN/Reshape_2"
  input: "CNN/strided_slice_3/stack"
  input: "CNN/strided_slice_3/stack_1"
  input: "CNN/strided_slice_3/stack_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/zeros_2/shape_as_tensor"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 3072
      }
    }
  }
}
node {
  name: "CNN/zeros_2/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 0.0
      }
    }
  }
}
node {
  name: "CNN/zeros_2"
  op: "Fill"
  input: "CNN/zeros_2/shape_as_tensor"
  input: "CNN/zeros_2/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/mul_2"
  op: "Mul"
  input: "CNN/zeros_2"
  input: "CNN/strided_slice_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/Const_4"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 0
      }
    }
  }
}
node {
  name: "CNN/Sum_2"
  op: "Sum"
  input: "CNN/mul_2"
  input: "CNN/Const_4"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_2/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/gradients_2/grad_ys_0"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_FLOAT
        tensor_shape {
        }
        float_val: 1.0
      }
    }
  }
}
node {
  name: "CNN/gradients_2/Fill"
  op: "Fill"
  input: "CNN/gradients_2/Shape"
  input: "CNN/gradients_2/grad_ys_0"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "index_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/Sum_2_grad/Reshape/shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 1
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/Sum_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_2/Fill"
  input: "CNN/gradients_2/CNN/Sum_2_grad/Reshape/shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/Sum_2_grad/Const"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 1
          }
        }
        int_val: 3072
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/Sum_2_grad/Tile"
  op: "Tile"
  input: "CNN/gradients_2/CNN/Sum_2_grad/Reshape"
  input: "CNN/gradients_2/CNN/Sum_2_grad/Const"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tmultiples"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/mul_2_grad/Mul"
  op: "Mul"
  input: "CNN/gradients_2/CNN/Sum_2_grad/Tile"
  input: "CNN/strided_slice_3"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/mul_2_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients_2/CNN/Sum_2_grad/Tile"
  input: "CNN/zeros_2"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/strided_slice_3_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 2
          }
        }
        tensor_content: "\001\000\000\000\000\014\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/strided_slice_3_grad/StridedSliceGrad"
  op: "StridedSliceGrad"
  input: "CNN/gradients_2/CNN/strided_slice_3_grad/Shape"
  input: "CNN/strided_slice_3/stack"
  input: "CNN/strided_slice_3/stack_1"
  input: "CNN/strided_slice_3/stack_2"
  input: "CNN/gradients_2/CNN/mul_2_grad/Mul_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "begin_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "ellipsis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "end_mask"
    value {
      i: 2
    }
  }
  attr {
    key: "new_axis_mask"
    value {
      i: 0
    }
  }
  attr {
    key: "shrink_axis_mask"
    value {
      i: 1
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/Reshape_2_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/Reshape_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_2/CNN/strided_slice_3_grad/StridedSliceGrad"
  input: "CNN/gradients_2/CNN/Reshape_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\001\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
          }
        }
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/BroadcastGradientArgs"
  op: "BroadcastGradientArgs"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Shape"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Sum"
  op: "Sum"
  input: "CNN/gradients_2/CNN/Reshape_2_grad/Reshape"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/BroadcastGradientArgs"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Reshape"
  op: "Reshape"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Sum"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Shape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Sum_1"
  op: "Sum"
  input: "CNN/gradients_2/CNN/Reshape_2_grad/Reshape"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/BroadcastGradientArgs:1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tidx"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "keep_dims"
    value {
      b: false
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Reshape_1"
  op: "Reshape"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Sum_1"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Shape_1"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tshape"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/kernel/read"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_2/CNN/dpt_decode/add_2_grad/Reshape"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad_grad/MirrorPadGrad"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_5/kernel/read"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad_grad/MirrorPadGrad"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_pyr/pyr_2/MirrorPad_grad/MirrorPadGrad"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  op: "MirrorPadGrad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/Conv2D_grad/Conv2DBackpropInput"
  input: "CNN/dpt_decode/dpt_dec/conv_5/MirrorPad/paddings"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "Tpaddings"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "mode"
    value {
      s: "SYMMETRIC"
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Rank"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
        }
        int_val: 4
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/mod"
  op: "FloorMod"
  input: "CNN/dpt_decode/dpt_dec/concat_2/concat/axis"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Rank"
  attr {
    key: "T"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  op: "Const"
  attr {
    key: "dtype"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "value"
    value {
      tensor {
        dtype: DT_INT32
        tensor_shape {
          dim {
            size: 4
          }
        }
        tensor_content: "\001\000\000\000\200\000\000\0000\000\000\000@\000\000\000"
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  op: "ConcatOffset"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/mod"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  attr {
    key: "N"
    value {
      i: 3
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
  op: "Slice"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_1"
  op: "Slice"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:1"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_1"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  op: "Slice"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_5/MirrorPad_grad/MirrorPadGrad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/ConcatOffset:2"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Shape_2"
  attr {
    key: "Index"
    value {
      type: DT_INT32
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul"
  op: "Mul"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  input: "CNN/img_decompose/img_unet/dec/conv_5/LeakyRelu"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul_1"
  op: "Mul"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice_2"
  input: "CNN/dpt_decode/dpt_dec/conv_4/Identity"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
}
node {
  name: "CNN/gradients_2/AddN"
  op: "AddN"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/mul_2_grad/Mul"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "_class"
    value {
      list {
        s: "loc:@CNN/gradients_2/CNN/dpt_decode/dpt_dec/concat_2/concat_grad/Slice"
      }
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_4/BiasAdd_grad/BiasAddGrad"
  op: "BiasAddGrad"
  input: "CNN/gradients_2/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN"
  op: "ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  attr {
    key: "N"
    value {
      i: 2
    }
  }
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "out_type"
    value {
      type: DT_INT32
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropInput"
  op: "Conv2DBackpropInput"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN"
  input: "CNN/dpt_decode/dpt_dec/conv_4/kernel/read"
  input: "CNN/gradients_2/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "padding"
    value {
      s: "VALID"
    }
  }
  attr {
    key: "strides"
    value {
      list {
        i: 1
        i: 1
        i: 1
        i: 1
      }
    }
  }
  attr {
    key: "use_cudnn_on_gpu"
    value {
      b: true
    }
  }
}
node {
  name: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/Conv2DBackpropFilter"
  op: "Conv2DBackpropFilter"
  input: "CNN/dpt_decode/dpt_dec/conv_4/MirrorPad"
  input: "CNN/gradients_2/CNN/dpt_decode/dpt_dec/conv_4/Conv2D_grad/ShapeN:1"
  input: "CNN/gradients_2/AddN"
  attr {
    key: "T"
    value {
      type: DT_FLOAT
    }
  }
  attr {
    key: "data_format"
    value {
      s: "NCHW"
    }
  }
  attr {
    key: "dilations"
    value {
      }