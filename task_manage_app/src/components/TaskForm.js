import React from 'react';
import { useDispatch, useSelector } from 'react-redux';
import { Form, Input, Button, Select, DatePicker } from 'antd';
import moment from 'moment';

import { addTask } from '../features/tasks/tasksSlice';

const { Option } = Select;
const { TextArea } = Input;

const TaskForm = () => {

  const [form] = Form.useForm();
  const dispatch = useDispatch();

  // Handler for form submission
  const onFinish = (values) => {
    // Since the DatePicker returns a moment object, format it to a string
    const formattedValues = {
      ...values,
      categorie: values.category,
      dueDate: values.dueDate ? values.dueDate.format('YYYY-MM-DD') : '',
    };
    console.log(formattedValues);
    dispatch(addTask(formattedValues));
    form.resetFields(); // Reset form fields after submission
  };
  const categories = useSelector(state => state.tasks.categories);
  // console.log('hi' ,categories.map(category => ({ label: category, value: category, })));

  // Define a function to disable dates before today
  const disabledDate = (current) => {
    // Can not select days before today
    return current && current < moment().startOf('day');
  };

  return (
    
    <Form
    form={form} // Associate the form instance with the Form component
    layout="vertical" 
    onFinish={onFinish} // Specify the function to call when the form is submitted
    initialValues={{
      priority: '20', // Set the default value for the priority field
    }}
  >
    <Form.Item
      label="Title"
      name="title"
      rules={[{ required: true, message: 'Please input the title!' }]} // Add a rule to make this field required
    >
      <Input placeholder="Title" />
    </Form.Item>

    <Form.Item
      label="Description"
      name="description"
    >
      <TextArea rows={4} placeholder="Description" />
    </Form.Item>

    <Form.Item
      label="Category"
      name="category"
      rules={[{ required: true, message: 'Please input the category!' }]}
    >
      <Select
        mode="tags"
        style={{ width: '100%' }}
        placeholder="Select or type a new category"
        popupMatchSelectWidth={false}
        maxCount={1}  // Limit the number of selected categories to 1, only take the first one, or the strategy is quite puzzling; Besides, this is category, not tag
        options={categories.map(category => ({
          label: category, 
          value: category, 
        }))}
      />
    </Form.Item>

    <Form.Item
      label="Priority"
      name="priority"
      rules={[{ required: true, message: 'Please select a priority!' }]} // Add a rule to make this field required
    >
      <Select placeholder="Select a priority">
        <Option value="10">High</Option>
        <Option value="20">Medium</Option>
        <Option value="30">Low</Option>
      </Select>
    </Form.Item>

    <Form.Item
      label="Due Date"
      name="dueDate"
    >
      <DatePicker format="YYYY-MM-DD" disabledDate={disabledDate} />
    </Form.Item>

    <Form.Item>
      <Button type="primary" htmlType="submit">
        Add Task
      </Button>
    </Form.Item>
  </Form>
  );
};

export default TaskForm;
