{-# LANGUAGE ForeignFunctionInterface #-}
import Foreign
import Foreign.C

newtype DynamicsWorld = DynamicsWorld { unDynamicsWorld :: Ptr DynamicsWorld }

foreign import ccall "createDynamicsWorld" createDynamicsWorld :: IO DynamicsWorld
foreign import ccall "testBullet" testBullet :: DynamicsWorld -> IO Int

main = do
    putStrLn "Testing bullet..."
    dynamicsWorld <- createDynamicsWorld 
    testBullet dynamicsWorld
    putStrLn "Done!"