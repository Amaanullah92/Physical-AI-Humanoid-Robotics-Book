import React, { lazy, Suspense } from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type { WrapperProps } from '@docusaurus/types';

type Props = WrapperProps<typeof LayoutType>;

const ChatWidget = lazy(() => import('@site/src/components/ChatWidget'));

export default function LayoutWrapper(props: Props): JSX.Element {
  return (
    <>
      <Layout {...props} />
      <Suspense fallback={<div>Loading Chat...</div>}>
        <ChatWidget />
      </Suspense>
    </>
  );
}
