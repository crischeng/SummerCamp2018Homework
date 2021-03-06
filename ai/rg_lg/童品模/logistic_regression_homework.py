#!/usr/bin/python2.7
#coding:utf-8

'''
A logistic regression learning algorithm example using TensorFlow library.
This example is using the MNIST database of handwritten digits
(http://yann.lecun.com/exdb/mnist/)
Author:
'''

from __future__ import print_function

import tensorflow as tf

# Import MNIST data
from tensorflow.examples.tutorials.mnist import input_data
mnist = input_data.read_data_sets("../Mnist_data/", one_hot=True)
print(mnist)

# Parameters setting
learning_rate = 0.01
training_epochs = 25 # 训练迭代的次数
batch_size = 100   # 一次输入的样本
display_step = 1

# set the tf Graph Input & set the model weights
# TO DO
x=tf.placeholder(dtype=tf.float32,shape=[None,784],name='input_x')
y=tf.placeholder(dtype=tf.float32,shape=[None,10],name='input_y')

#set models weights,bias
W=tf.Variable(tf.zeros([784,10]))
b=tf.Variable(tf.zeros([10]))


# Construct the model
# TO DO
print(tf.matmul(x,W)+b)

pred=tf.nn.softmax(tf.matmul(x,W)+b)  # 归一化,the possibility of getting the right value

print(pred)
# Minimize error using cross entropy & set the gradient descent
# TO DO
cost=tf.reduce_mean(-tf.reduce_sum(y*tf.log(pred),reduction_indices=1)) #交叉熵，reducion_indices=1横向求和
optimizer=tf.train.GradientDescentOptimizer(learning_rate).minimize(cost)


# Initialize the variables (i.e. assign their default value)
init = tf.global_variables_initializer()

# Start training
with tf.Session() as sess:

    # Run the initializer
    #
    sess.run(init)

    # Training cycle
    for epoch in range(training_epochs):
        avg_cost = 0.
        total_batch = int(mnist.train.num_examples/batch_size)
        # Loop over all batches
        for i in range(total_batch):
            batch_xs, batch_ys = mnist.train.next_batch(batch_size)
            # Run optimization op (backprop) and cost op (to get loss value)
            _, c = sess.run([optimizer, cost], feed_dict={x: batch_xs,
                                                          y: batch_ys})
            # Compute average loss
            avg_cost += c / total_batch
        # Display logs per epoch step
        if (epoch+1) % display_step == 0:
            print("Epoch:", '%04d' % (epoch+1), "cost=", "{:.9f}".format(avg_cost))

    print("Optimization Finished!")

    # Test model
    correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(y, 1))
    # Calculate accuracy
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
    print("Accuracy:", accuracy.eval({x: mnist.test.images, y: mnist.test.labels}))