import React, {useEffect} from 'react';
import { useSelector, useDispatch } from 'react-redux';
import { notification, Flex, Select, Input } from 'antd';
import moment from 'moment';
import { setSort, selectSortedTasks, reorderTasks, setFilterStatus, setFilterCategory, setSearchTerm } from '../features/tasks/tasksSlice';
import Task from './TaskEntry'; // Import the Task component

const TaskList = () => {
  const dispatch = useDispatch();
  const tasks = useSelector(selectSortedTasks);
  const categories = useSelector(state => state.tasks.categories);


  // Handler to change sort criterion based on user selection
  const handleSortChange = (value) => {
    dispatch(setSort(value));
  };

  const moveTask = (dragIndex, hoverIndex) => {
    // Create a new array representing the new order of tasks
    let newOrder = [...tasks];
    const [removed] = newOrder.splice(dragIndex, 1);
    newOrder.splice(hoverIndex, 0, removed);
    
    // Dispatch an action with newOrder containing the updated array of tasks
    dispatch(reorderTasks(newOrder.map(task => task.id)));
  };

  useEffect(() => {
    const notifyApproachingDueDates = (tasks) => {
      const now = moment();
      tasks.forEach(task => {
        const dueDate = moment(task.dueDate, "YYYY-MM-DD");
        if (dueDate.isAfter(now) && dueDate.diff(now, 'hours') <= 24 && !task.completed) {
          // This task is within 24 hours of its due date and is not completed
          triggerNotification(task);
        }
      });
    };
    
    const triggerNotification = (task) => {
      notification.open({
        message: 'Upcoming Task Due Date',
        description: `The task "${task.title}" is due on ${task.dueDate}.`,
        duration: 3, // Keep the notification visible until closed manually
      });
    };

    // Call the function to check for tasks nearing their due dates
    notifyApproachingDueDates(tasks);
  }, [tasks]);

  return (
    <div>
      <Flex gap="10px" wrap="wrap">
        <Select onChange={handleSortChange} defaultValue="priority" options={[
          { value: "priority", label: <span>Sort by Priority</span> },
          { value: "dueDate", label: <span>Sort by Due Date</span> },
          { value: "custom", label: <span>Custom Sort</span> }
        ]} />
        <Select onChange={value => dispatch(setFilterStatus(value))} defaultValue="all" options={[
          { value: "all", label: <span>All Tasks</span> },
          { value: "completed", label: <span>Completed</span> },
          { value: "incomplete", label: <span>Incomplete</span> }
        ]} />
        <Select onChange={value => dispatch(setFilterCategory(value))} defaultValue="all" style={{minWidth: "100px"}} options={[
          { value: "all", label: <span>All Categories</span> },
          ...categories.map(category => ({ value: category, label: <span>{category}</span> }))
        ]} />
        <Input
          placeholder="Search tasks by title..."
          onChange={(e) => dispatch(setSearchTerm(e.target.value))}
          style={{ minWidth: '100px', maxWidth: '300px'}}
        />
      </Flex>


      <ul>
      {tasks.map((task, index) => (
          <Task key={task.id} id={task.id} title={task.title} index={index} moveTask={moveTask} completed={task.completed} dueDate={task.dueDate} priority={task.priority} description={task.description} />

      ))}
      </ul>
    </div>

  );
};

export default TaskList;
