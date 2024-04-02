import React, { useRef } from 'react';
import { useDrag, useDrop } from 'react-dnd';
import { useDispatch } from 'react-redux'; 
import { Card, Checkbox, Button, Flex, Typography, Tag } from 'antd';
import { DeleteOutlined, MoreOutlined } from '@ant-design/icons';
import { toggleTaskCompleted, deleteTask } from '../features/tasks/tasksSlice'; 


const {Text} = Typography;
const Task = ({ id, title, index, moveTask, completed, dueDate, priority, description }) => {
  const dispatch = useDispatch();
  const ref = useRef(null);

  const isOverdue = !completed && new Date(dueDate) < new Date();
  // console.log(isOverdue, dueDate, new Date(dueDate),);

  const [, drop] = useDrop({
    accept: 'task',
    hover(item, monitor) {
      if (!ref.current) {
        return;
      }
      const dragIndex = item.index;
      const hoverIndex = index;

      // Don't replace items with themselves
      if (dragIndex === hoverIndex) {
        return;
      }

      // Move the task in the list
      moveTask(dragIndex, hoverIndex);

      // Update the index for dragged item to prevent flickering
      item.index = hoverIndex;
    },
  });

  const [{ isDragging }, drag] = useDrag({
    type: 'task',
    item: () => {
      return { id, index };
    },
    collect: (monitor) => ({
      isDragging: !!monitor.isDragging(),
    }),
  });

  drag(drop(ref));

  // Handler for toggling task completion
  const handleToggleCompleted = () => {
    dispatch(toggleTaskCompleted(id));
  };

  // Handler for deleting a task
  const handleDeleteTask = () => {
    dispatch(deleteTask(id));
  };


  return (
    <Card
    ref={ref}
    size='small'
    hoverable={true}
    style={
      {
        marginBottom: '10px',
        opacity: isDragging ? 0.5 : 1,
        textDecoration: completed ? 'line-through' : 'none',
        color: isOverdue ? 'red' : (completed ? 'gray' : 'black'),  // Add color style based on task status
      }
        }
        >
      <Flex>
        <MoreOutlined />
        <Checkbox
          checked={completed}
          onChange={handleToggleCompleted}
          style={{ marginRight: '10px', width:'5%' }}
        />
        <Text
          style={{ width: '20%', color: isOverdue ? 'red' : (completed ? 'gray' : 'black'), }}
        >{title}</Text>
        <Text style={{ width: '70%', color: 'gray' }}>{description}</Text>
        
        <Tag color={priority === '10' ? 'red' : (priority === '20' ? 'orange' : 'green')}>
          {priority === '10' ? 'High' : (priority === '20' ? 'Medium' : 'Low')}
        </Tag>
        <Tag color={ isOverdue ? 'red' : (completed ? 'gray' : 'blue')}>{dueDate}</Tag>
        
        <div style={{ width: '5%' }}>
        <Button type="default" shape="round" size="small" icon={<DeleteOutlined/>} onClick={handleDeleteTask}></Button>
        </div>
        
      </Flex>


      {/* <button onClick={handleDeleteTask} style={{ marginLeft: '10px' }}>Delete</button> */}
    </Card>

  );
};
export default Task;