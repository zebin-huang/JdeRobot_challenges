import { createSlice, nanoid } from '@reduxjs/toolkit';
import { createSelector } from 'reselect';

// Adjusting initialState to an object to include sort property
const initialState = {
  tasks: [],
  sort: 'priority', // Default sort state
  filter: {
    status: 'all', // all, completed, incomplete
    category: 'all', // all or specific category
    searchTerm: '', // search term to filter tasks
  },
  categories: [],
};

export const tasksSlice = createSlice({
  name: 'tasks',
  initialState,
  reducers: {
    addTask: {
      reducer(state, action) {
        const { category } = action.payload;
        
        // Add the category if it doesn't already exist, category is an array with one element, use [0]
        if (!state.categories.includes(category[0])) {
          state.categories.push(category[0]);
        }
        state.tasks.push(action.payload);
      },
      prepare({ title, description, category, priority, dueDate }) {
        return { 
          payload: { 
            id: nanoid(), 
            title, 
            description, 
            category, 
            priority, 
            dueDate,
            completed: false 
          } 
        };
      },
    },
    // Set sort criterion
    setSort: (state, action) => {
      state.sort = action.payload;
    },

    // Reorder tasks based on new order
    reorderTasks: (state, action) => {
      const newOrder = action.payload;
      state.tasks = newOrder.map(id => state.tasks.find(task => task.id === id));
    },

    // Toggle task completion status
    toggleTaskCompleted: (state, action) => {
      const task = state.tasks.find(task => task.id === action.payload);
      if (task) {
        task.completed = !task.completed;
      }
    },

    // Delete a task
    deleteTask: (state, action) => {
      const taskId = action.payload;
      state.tasks = state.tasks.filter(task => task.id !== taskId);
    },

    // Set filter status
    setFilterStatus: (state, action) => {
      state.filter.status = action.payload;
    },

    // Set filter category
    setFilterCategory: (state, action) => {
      console.log(action.payload);
      state.filter.category = action.payload;
    },

    // Set search term
    setSearchTerm: (state, action) => {
      state.filter.searchTerm = action.payload;
    },
  },
});

export const { addTask, setSort, reorderTasks, toggleTaskCompleted, deleteTask, setFilterCategory, setFilterStatus, setSearchTerm } = tasksSlice.actions;

// Selector to get tasks from different properties of the state
export const selectSortedTasks = createSelector(
  [state => state.tasks.tasks, state => state.tasks.sort, state => state.tasks.filter],

  (tasks, sort, filter) => {
    let filteredTasks = tasks;
    if (filter.status !== 'all') {
      const isCompleted = filter.status === 'completed';
      filteredTasks = filteredTasks.filter(task => task.completed === isCompleted);
    }
    if (filter.category !== 'all') {
      console.log(tasks, filter.category);
      filteredTasks = filteredTasks.filter(task => task.category.includes(filter.category));
    }
    if (filter.searchTerm) {
      filteredTasks = filteredTasks.filter(task =>
        task.title.toLowerCase().includes(filter.searchTerm.toLowerCase())
      );
    }

    switch (sort) {
      case 'priority':
        return [...filteredTasks].sort((a, b) => {
          // use ...filteredTasks to create a new array to avoid mutating the original array
          const priorityComparison = a.priority - b.priority;
          if (priorityComparison !== 0) return priorityComparison;
          // If the priorities are equal, sort by id
          return a.id < b.id ? -1 : 1;
        }
        
        );
      case 'dueDate':
        return [...filteredTasks].sort((a, b) => new Date(a.dueDate) - new Date(b.dueDate));
      case 'custom':
        // Assuming tasks are already in the correct order for 'custom'
        return filteredTasks;
      default:
        return filteredTasks;
    }
  }
);

export default tasksSlice.reducer;