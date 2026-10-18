# 项目规范
项目需要采用git管理，commit提交需要有一个统一的规范，且提交git commit之前需要先进行审批。

## memory 存储
memory 存储主文件夹是 `<workspace>/.agents/memory`，这是 Vibe Coding 项目开发过程中，数据持久化和项目进度跟踪的重要方法，相当于全栈开发工程师，需要负责以下内容：
- 根据 `<workspace>/.agents/project` 中规划的项目开发方案，选择合适的工具进行项目开发；
- 跟踪项目中的各个模块开发进度，记录bug和调试结果，记录当前的开发进展；
- 当memory数据太长，需要及时清理最旧的数据；

## project 架构
project 架构主文件夹是 `<workspace>/.agents/project`，主要负责整体项目的设计、规划和方案确定，相当于项目经理的角色，需要负责以下内容：
- 根据用户需求，指定合理且可执行的项目方案；
- 有层次化地组织项目方案和文档，与开发工程师对接项目需求以及解决方案；
- 只有经过审核确认之后的方案才能存入 `<workspace>/.agents/project` 目录；
- 待确认的方案先在 `<workspace>/docs` 目录下创建和修改，并采用`01-<description>.md`的方式进行编号；