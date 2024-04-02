import React from 'react';
import { ConfigProvider, Layout, Typography, Card, Divider } from 'antd';
import TaskForm from './components/TaskForm';
import TaskList from './components/TaskList';
import { DndProvider } from 'react-dnd';
import { HTML5Backend } from 'react-dnd-html5-backend';


const { Header, Content } = Layout;
const { Title } = Typography;

function App() {
  return (
    <ConfigProvider
    // theme={{
    //   algorithm: [theme.defaultAlgorithm, theme.compactAlgorithm],
    // }}
    >
      <DndProvider backend={HTML5Backend}>
      <Layout>
          <Header style={{ background: '#fff', padding: '0 20px' }}>
            <Title level={1} style={{ margin: '14px 0' }}>Task Manager</Title>
          </Header>
          <Content style={{ padding: '20px' }}>
            <div style={{ background: '#fff', padding: '20px', minHeight: 280 }}>
              <Card
              title="New Task"
              >

                <TaskForm />
              </Card>
              <Divider />
              <Card 
              title="Tasks"
              >
                <TaskList />
              </Card>
            </div>
          </Content>
        </Layout>
      </DndProvider>
    </ConfigProvider>
  );
}

export default App;
