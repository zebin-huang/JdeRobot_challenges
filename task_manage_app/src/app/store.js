import { persistStore, persistReducer } from 'redux-persist';
import storage from 'redux-persist/lib/storage'; // use localStorage by default

import { configureStore } from '@reduxjs/toolkit';
import rootReducer from './rootReducer';

const persistConfig = {
  key: 'root',
  storage,
};

// use the persistReducer to persist the rootReducer
const persistedReducer = persistReducer(persistConfig, rootReducer);

export const store = configureStore({
  reducer: persistedReducer, 
});

export const persistor = persistStore(store);
